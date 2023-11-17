#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <asm/msr.h>
#include <linux/version.h>

#include "nmttcan.h"


#define DRV_MODULENAME                    ttcan
#define DRV_STRID                         "ttcan"
#define DRV_STRDESC                       "TTCAN"

#define DRV_VERSION_AND_DATE              0x32031023
#define DRV_VERSION                       "version: 3.2 date: 03.10.2023"

#define MAX_NUM_BOARDS                    32

#define DRV_IOCTL_CONCURRENT_MAX          16

#define DRV_INITIALIZED                   1
#define DRV_READSNAPSHOT                  2
#define DRV_GOTSNAPSHOT                   4

#ifdef INIT_COMPLETION
#define reinit_completion(x) INIT_COMPLETION(*(x))
#endif

#define nofels(A)                         (sizeof(A)/sizeof(*(A)))
#define STRNCPY(dst, src)                 do { strncpy(dst, src, sizeof(dst) - 1); dst[sizeof(dst) - 1] = '\0'; } while (0)
#define STRNCAT(s, append)                strncat(s, append, sizeof(s) - strlen(s) - 1)
#define SNPRINTF(str, format, ...)        snprintf(str, sizeof(str), format , ##__VA_ARGS__)
#define VSNPRINTF(str, format, ap, ...)   vsnprintf(str, sizeof(str), format, ap , ##__VA_ARGS__)

#define INL_AND_OUTL(pdev, offset, mask)  OUTL(pdev, offset, INL(pdev, offset) & (mask))
#define INL_OR_OUTL(pdev, offset, mask)   OUTL(pdev, offset, INL(pdev, offset) | (mask))
#define INL_AND_OR_OUTL(pdev, offset, maskand, maskor) \
                                          OUTL(pdev, offset, (INL(pdev, offset) & (maskand)) | (maskor))

#define DRV_LOCK_VARS()                   unsigned long drv_lock_flags
#define DRV_LOCK(pdev)                    spin_lock_irqsave(&((pdev)->lock), drv_lock_flags)
#define DRV_UNLOCK(pdev)                  spin_unlock_irqrestore(&((pdev)->lock), drv_lock_flags)

#define DRV_LOCK_CAN(pdev, ch)            down_interruptible(&((pdev)->sem_can[ch]))
#define DRV_UNLOCK_CAN(pdev, ch)          up(&((pdev)->sem_can[ch]))

#define DRV_LOCK_CAN_BUF(pdev, ch, nbuf)   down_interruptible(&((pdev)->sem_can_buf[ch][nbuf]))
#define DRV_UNLOCK_CAN_BUF(pdev, ch, nbuf) up(&((pdev)->sem_can_buf[ch][nbuf]))

#define DRV_LOCK_READER_VARS()            unsigned long drv_lock_reader_flags
#define DRV_LOCK_READER(pdev, ch)         spin_lock_irqsave(&((pdev)->lock_reader[ch]), drv_lock_reader_flags)
#define DRV_UNLOCK_READER(pdev, ch)       spin_unlock_irqrestore(&((pdev)->lock_reader[ch]), drv_lock_reader_flags)

#define DRV_LOCK_READ(pdev, ch)           down_interruptible(&((pdev)->sem_read[ch]))
#define DRV_UNLOCK_READ(pdev, ch)         up(&((pdev)->sem_read[ch]))

#define _DRV_TUNABLE_INT(modulename, var, value) \
  static int drv_##var = value; \
  module_param_named(modulename##_##var, drv_##var, int, S_IRUGO)
#define DRV_TUNABLE_INT(modulename, var, value) _DRV_TUNABLE_INT(modulename, var, value)


typedef struct _TTXBUF
{
  int      wrote;
  uint8_t  prio;
  uint32_t sid;
  uint32_t eid;
  uint8_t  data[8];
  uint32_t nsize;
} TTXBUF;

typedef struct _TDEV
{
  uint8_t __iomem*        pbar;
  uint64_t                barlen;
  struct cdev             cdev;
  struct device*          pcdev_device;
  struct pci_dev*         pdevice;
  u_int                   unit;
  char                    devname[32];
  spinlock_t              lock;
  spinlock_t              lock_reader[2];
  struct semaphore        sem_can[2];
  struct semaphore        sem_can_buf[2][CAN_BUFS];
  struct semaphore        sem_read[2];
  struct completion       acs_completion[2];
  struct completion       xmtd_completion[2][CAN_BUFS];
  struct completion       timer_completion[2];
  struct completion       read_completion[2];

  uint16_t                vendorid;
  uint16_t                deviceid;
  uint8_t                 revisionid;
  uint32_t                int_mask;
  int                     busy;
  uint32_t                flags;

  size_t                  dma_size;
  dma_addr_t              dma_paddr;
  uint8_t*                dma_vaddr;
  uint8_t*                ch_dma_vaddr[2];

  int                     debug;
  int                     debug_regs;
  int                     debug_regs_can;
  int                     debug_dma;
  int                     debug_intr;
  int                     debug_can;
  int                     debug_send;
  int                     debug_wait;
  int                     debug_halt_on_timeout;
  int                     disable_autorts;

  volatile uint32_t       dma_wrptr[2];
  volatile uint32_t       dma_rdptr[2];
  volatile uint32_t       reader_sleeping[2];
  volatile uint32_t       reader_size[2];
  TTXBUF                  txbuf[2][CAN_BUFS];
  int                     halted;

  struct mutex            mtxR;
  struct mutex            mtxT1;
  struct mutex            mtxT2;

  uint32_t*       dma_wr[2];
  uint32_t*       dma_rd[2];
} TDEV;


static int drv_probe(struct pci_dev* pdevice, const struct pci_device_id* pid);
static void drv_remove(struct pci_dev* pdevice);
static void drv_shutdown(struct pci_dev* pdevice);

static int drv_open(struct inode* pinode, struct file* pfile);
static int drv_close(struct inode* pinode, struct file* pfile);
static ssize_t drv_read(struct file* pfile, char __user* pbuf, size_t buflen, loff_t* offset);
static ssize_t drv_write(struct file* pfile, const char __user* pbuf, size_t buflen, loff_t* offset);
static int drv_mmap(struct file* pfile, struct vm_area_struct* pvma);
static long drv_ioctl(struct file* pfile, unsigned cmd, unsigned long param);

static irqreturn_t drv_intr_handler(int irq, void* arg);


MODULE_AUTHOR("Novomar, Ltd.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TTCAN Driver");
MODULE_VERSION(DRV_VERSION);

static const struct pci_device_id drv_pci_ids[] =
{
  { PCI_DEVICE(NM_TTCAN_VENDOR_ID, NM_TTCAN_DEVICE_ID) },
  { 0 }
};

MODULE_DEVICE_TABLE(pci, drv_pci_ids);

static const char drv_name[] = DRV_STRID;
static const char drv_version[] = DRV_VERSION;

static struct pci_driver ttcan_driver =
{
  .name = drv_name,
  .id_table = drv_pci_ids,
  .probe = drv_probe,
  .remove = drv_remove,
  .shutdown = drv_shutdown
};

static const struct file_operations drv_fops =
{
  .owner          = THIS_MODULE,
  .open           = drv_open,
  .release        = drv_close,
  .read           = drv_read,
  .write          = drv_write,
  .unlocked_ioctl = drv_ioctl,
  .mmap           = drv_mmap,
};

static struct class* drv_cdev_class;
static int drv_cdev_major;
static TDEV* pdevs[MAX_NUM_BOARDS];

DRV_TUNABLE_INT(DRV_MODULENAME, debug, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_regs, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_regs_can, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_dma, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_intr, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_can, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_send, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_wait, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, debug_halt_on_timeout, 0);
DRV_TUNABLE_INT(DRV_MODULENAME, disable_autorts, 0);

static const uint32_t reg_ctrl[2] = { OFFSET_CAN1_CTRL_REG_PCI_mPCIe_CAN, OFFSET_CAN2_CTRL_REG_PCI_mPCIe_CAN };
static const uint32_t reg_acs[2] = { OFFSET_CAN1_ACS_REG_mPCIe_CAN, OFFSET_CAN2_ACS_REG_mPCIe_CAN };
static const uint32_t reg_buf[2] = { OFFSET_CAN1_DATA_BUF_mPCIe_CAN, OFFSET_CAN2_DATA_BUF_mPCIe_CAN };
static const uint32_t reg_trig[2][CAN_BUFS] =
{
  { OFFSET_CAN1_TX1_TRIG, OFFSET_CAN1_TX2_TRIG, OFFSET_CAN1_TX3_TRIG },
  { OFFSET_CAN2_TX1_TRIG, OFFSET_CAN2_TX2_TRIG, OFFSET_CAN2_TX3_TRIG }
};
static const uint32_t reg_trig_epoch[2][CAN_BUFS] =
{
  { OFFSET_CAN1_TX1_TRIG_EPOCH, OFFSET_CAN1_TX2_TRIG_EPOCH, OFFSET_CAN1_TX3_TRIG_EPOCH },
  { OFFSET_CAN2_TX1_TRIG_EPOCH, OFFSET_CAN2_TX2_TRIG_EPOCH, OFFSET_CAN2_TX3_TRIG_EPOCH }
};
static const uint32_t reg_trig_ctrl[2] = { OFFSET_CAN1_TRIG_CTRL, OFFSET_CAN2_TRIG_CTRL };
static const uint32_t reg_timer_trsh[2] = { OFFSET_CAN1_TIMER_TRSH, OFFSET_CAN2_TIMER_TRSH };
static const uint32_t reg_timer_ceed[2] = { OFFSET_CAN1_TIMER_CEED, OFFSET_CAN2_TIMER_CEED };
static const uint32_t reg_epoch_ceed[2] = { OFFSET_CAN1_EPOCH_CEED, OFFSET_CAN2_EPOCH_CEED };
static const uint32_t reg_timer_ctrl[2] = { OFFSET_CAN1_TIMER_CTRL, OFFSET_CAN2_TIMER_CTRL };
static const uint32_t reg_timer[2] = { OFFSET_CAN1_TIMER, OFFSET_CAN2_TIMER };
static const uint32_t reg_timer_epoch[2] = { OFFSET_CAN1_TIMER_EPOCH, OFFSET_CAN2_TIMER_EPOCH };
static const uint32_t reg_int_trig[2] = { OFFSET_CAN1_INT_TRIG, OFFSET_CAN2_INT_TRIG };
static const uint32_t reg_int_trig_epoch[2] = { OFFSET_CAN1_INT_TRIG_EPOCH, OFFSET_CAN2_INT_TRIG_EPOCH };
static const uint32_t reg_timeout_absolute[2] = { OFFSET_CAN1_TIMEOUT_ABSOLUTE, OFFSET_CAN2_TIMEOUT_ABSOLUTE };
static const uint32_t reg_timeout_interval[2] = { OFFSET_CAN1_TIMEOUT_INTERVAL, OFFSET_CAN2_TIMEOUT_INTERVAL };
static const uint32_t reg_dma_index[2] = { OFFSET_DMA1_INDEX_REG_TTCAN, OFFSET_DMA2_INDEX_REG_TTCAN };
static const uint32_t reg_dma_rd_index[2] = { OFFSET_DMA1_RD_INDEX_REG_TTCAN, OFFSET_DMA2_RD_INDEX_REG_TTCAN };
static const uint8_t can_reg_tx_buf[CAN_BUFS] = { TXB0CTRL, TXB1CTRL, TXB2CTRL };
static const uint8_t can_reg_rx_buf[2] = { RXB0CTRL, RXB1CTRL };
static const uint8_t can_reg_mask[2] = { RXM0SIDH, RXM1SIDH };
static const uint8_t can_reg_filt[6] = { RXF0SIDH, RXF1SIDH, RXF2SIDH, RXF3SIDH, RXF4SIDH, RXF5SIDH };
static const uint8_t epoch_mask[9] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF };

static const char* drv_devtype_str[] = { "mPCIe", "PCIe" };

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static int sym_dev_uevent(struct device *dev, struct kobj_uevent_env *env) {
#else
static int sym_dev_uevent(const struct device *dev, struct kobj_uevent_env *env) {
#endif
    add_uevent_var(env, "DEVMODE=%#o", 0666);   // даём доступ user'у на запись/чтение
    return 0;
}


static int __init drv_init_module(void)
{
  int err;
  dev_t devt;

  pr_info("Novomar, Ltd. %s driver %s\n", DRV_STRDESC, drv_version);

  drv_cdev_class = class_create(THIS_MODULE, drv_name);
  if (IS_ERR(drv_cdev_class))
  {
    err = PTR_ERR(drv_cdev_class);
    pr_err("%s init_module: can't create character device class\n", drv_name);
    goto err_class;
  }
  drv_cdev_class->dev_uevent = sym_dev_uevent;

  devt = MKDEV(0, 0);
  err = alloc_chrdev_region(&devt, 0, MAX_NUM_BOARDS, drv_name);
  if (err != 0)
  {
    pr_err("%s init_module: can't allocate chrdev region\n", drv_name);
    goto err_chrdev_region;
  }

  drv_cdev_major = MAJOR(devt);

  err = pci_register_driver(&ttcan_driver);
  if (err != 0)
  {
    pr_err("%s init_module: can't register PCI driver\n", drv_name);
    goto err_driver;
  }

  return 0;

err_driver:
  unregister_chrdev_region(devt, MAX_NUM_BOARDS);
err_chrdev_region:
  class_destroy(drv_cdev_class);
err_class:
  return err;
}

module_init(drv_init_module);

static void __exit drv_exit_module(void)
{
  dev_t devt = MKDEV(drv_cdev_major, 0);

  pci_unregister_driver(&ttcan_driver);
  unregister_chrdev_region(devt, MAX_NUM_BOARDS);
  class_destroy(drv_cdev_class);
}

module_exit(drv_exit_module);

static int drv_printk(const char* level, char* devname, const char* fmt, ...)
{
  struct va_format vaf;
  va_list args;

  va_start(args, fmt);

  vaf.fmt = fmt;
  vaf.va = &args;

  return printk("%s%s: %pV", level, devname, &vaf);
}

#define drv_printf(pdev, fmt, arg...)       drv_printk(KERN_INFO, (pdev)->devname, fmt, ##arg)
#define drv_printf_warn(pdev, fmt, arg...)  drv_printk(KERN_WARNING, (pdev)->devname, fmt, ##arg)
#define drv_printf_err(pdev, fmt, arg...)   drv_printk(KERN_ERR, (pdev)->devname, fmt, ##arg)

static ssize_t drv_show_attr(struct device* dev, struct device_attribute* attr, char* buf)
{
  TDEV* pdev = (TDEV*)dev_get_drvdata(dev);
  if (strcmp("debug", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug);
  if (strcmp("debug_regs", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_regs);
  if (strcmp("debug_regs_can", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_regs_can);
  if (strcmp("debug_dma", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_dma);
  if (strcmp("debug_intr", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_intr);
  if (strcmp("debug_can", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_can);
  if (strcmp("debug_send", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_send);
  if (strcmp("debug_wait", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_wait);
  if (strcmp("debug_halt_on_timeout", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->debug_halt_on_timeout);
  if (strcmp("disable_autorts", attr->attr.name) == 0)
    return scnprintf(buf, PAGE_SIZE, "%d\n", pdev->disable_autorts);
  return -EINVAL;
}

static ssize_t drv_store_attr(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
  TDEV* pdev = (TDEV*)dev_get_drvdata(dev);
  int ret = -EINVAL;
  if (strcmp("debug", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug);
  else if (strcmp("debug_regs", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_regs);
  else if (strcmp("debug_regs_can", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_regs_can);
  else if (strcmp("debug_dma", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_dma);
  else if (strcmp("debug_intr", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_intr);
  else if (strcmp("debug_can", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_can);
  else if (strcmp("debug_send", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_send);
  else if (strcmp("debug_wait", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_wait);
  else if (strcmp("debug_halt_on_timeout", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->debug_halt_on_timeout);
  else if (strcmp("disable_autorts", attr->attr.name) == 0)
    ret = kstrtoint(buf, 10, &pdev->disable_autorts);
  if (ret != 0)
    return ret;
  return count;
}

static DEVICE_ATTR(debug,                 0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_regs,            0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_regs_can,        0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_dma,             0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_intr,            0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_can,             0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_send,            0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_wait,            0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(debug_halt_on_timeout, 0644, drv_show_attr, drv_store_attr);
static DEVICE_ATTR(disable_autorts,       0644, drv_show_attr, drv_store_attr);

static struct attribute* drv_attrs[] =
{
  &dev_attr_debug.attr,
  &dev_attr_debug_regs.attr,
  &dev_attr_debug_regs_can.attr,
  &dev_attr_debug_dma.attr,
  &dev_attr_debug_intr.attr,
  &dev_attr_debug_can.attr,
  &dev_attr_debug_send.attr,
  &dev_attr_debug_wait.attr,
  &dev_attr_debug_halt_on_timeout.attr,
  &dev_attr_disable_autorts.attr,
  NULL,
};

static struct attribute_group drv_attr_group =
{
  .name = "sysctl",
  .attrs = drv_attrs,
};

static inline uint32_t INL(TDEV* pdev, uint32_t offset)
{
  uint32_t value;

  value = ioread32(pdev->pbar + offset);
  if (pdev->debug_regs)
    drv_printf(pdev, "REG[%04XH]=>%08XH\n", offset, value);
  return value;
}

static inline void OUTL(TDEV* pdev, uint32_t offset, uint32_t value)
{
  iowrite32(value, pdev->pbar + offset);
  if (pdev->debug_regs)
    drv_printf(pdev, "REG[%04XH]<=%08XH\n", offset, value);
}

static void pci_express_get_negotiated_link_status(struct pci_dev* pdevice, uint8_t* plink_width, uint8_t* plink_speed)
{
  uint16_t pci_status;

  pci_read_config_word(pdevice, PCI_STATUS, &pci_status);
  if ((pci_status & PCI_STATUS_CAP_LIST) != 0)
  {
    uint8_t pci_capptr;

    pci_read_config_byte(pdevice, PCI_CAPABILITY_LIST, &pci_capptr);
    pci_capptr &= ~3;

    while (pci_capptr > 0)
    {
      uint8_t pci_cap_id;
      uint8_t pci_cap_nextptr;
      uint16_t link_status;

      pci_read_config_byte(pdevice, pci_capptr + PCI_CAP_LIST_ID, &pci_cap_id);
      pci_read_config_byte(pdevice, pci_capptr + PCI_CAP_LIST_NEXT, &pci_cap_nextptr);

      switch (pci_cap_id)
      {
        case PCI_CAP_ID_EXP :
          pci_read_config_word(pdevice, pci_capptr + 18, &link_status);
          *plink_width = (uint8_t)((link_status >> 4) & 0x3F);
          *plink_speed = (uint8_t)(link_status & 0x0F);
          return;
        default :
          break;
      }
      pci_capptr = pci_cap_nextptr & ~3;
    }
  }
  *plink_width = 0;
  *plink_speed = 0;
}

static void drv_init_ints(TDEV* pdev)
{
  uint32_t ch;

  for (ch = 0; ch < 2; ch ++)
    INL_OR_OUTL(pdev, reg_ctrl[ch], 1);
  pdev->int_mask = INT_TIM_CAN1_TTCAN | INT_TIM_CAN2_TTCAN |
                   INT_CAN_ACS1_TTCAN | INT_CAN_ACS2_TTCAN |
                   INT_MSG_RCVD1_TTCAN | INT_MSG_RCVD2_TTCAN |
                   INT_TIM_ABS1_TTCAN | INT_TIM_ABS2_TTCAN |
                   INT_TIM_ITV1_TTCAN | INT_TIM_ITV2_TTCAN |
                   INT_MSG_XMTD10_TTCAN | INT_MSG_XMTD11_TTCAN | INT_MSG_XMTD12_TTCAN |
                   INT_MSG_XMTD20_TTCAN | INT_MSG_XMTD21_TTCAN | INT_MSG_XMTD22_TTCAN |
                   INT_ERR_CAN1_TTCAN | INT_ERR_CAN2_TTCAN |
                   INT_MERR_CAN1_TTCAN | INT_MERR_CAN2_TTCAN;
  OUTL(pdev, OFFSET_INTERRUPT_MASK_REG_TTCAN, pdev->int_mask);
}

static void drv_uninit_ints(TDEV* pdev)
{
  uint32_t ch;
  pdev->int_mask = 0;
  OUTL(pdev, OFFSET_INTERRUPT_MASK_REG_TTCAN, 0);
  for (ch = 0; ch < 2; ch ++)
    OUTL(pdev, reg_ctrl[ch], 0);
}

static void bar_read(TDEV* pdev, uint32_t addr, uint8_t* pdata, uint32_t nsize)
{
  uint32_t i;
  uint32_t offset = 0;
  uint32_t dw;

  for (i = 0; i < nsize / sizeof(uint32_t); i ++)
  {
    *(uint32_t*)(pdata + offset) = INL(pdev, addr + offset);
    offset += sizeof(uint32_t);
  }
  switch (nsize % sizeof(uint32_t))
  {
    case 0 :
      break;
    case 1 :
      pdata[offset] = (uint8_t)INL(pdev, addr + offset);
      break;
    case 2 :
      *(uint16_t*)(pdata + offset) = (uint16_t)INL(pdev, addr + offset);
      break;
    case 3 :
      dw = INL(pdev, addr + offset);
      *(uint16_t*)(pdata + offset) = (uint16_t)dw;
      pdata[offset + 2] = (uint8_t)(dw >> 16);
      break;
    default :
      break;
  }
}

static void bar_write(TDEV* pdev, uint32_t addr, uint8_t* pdata, uint32_t nsize)
{
  uint32_t i;
  uint32_t offset = 0;
  uint32_t dw;

  for (i = 0; i < nsize / sizeof(uint32_t); i ++)
  {
    OUTL(pdev, addr + offset, *(uint32_t*)(pdata + offset));
    offset += sizeof(uint32_t);
  }
  switch (nsize % sizeof(uint32_t))
  {
    case 0 :
      break;
    case 1 :
      OUTL(pdev, addr + offset, pdata[offset]);
      break;
    case 2 :
      OUTL(pdev, addr + offset, *(uint16_t*)(pdata + offset));
      break;
    case 3 :
      dw = (pdata[offset + 2] << 16) | *(uint16_t*)(pdata + offset);
      OUTL(pdev, addr + offset, dw);
      break;
    default :
      break;
  }
}

static inline int can_wait(TDEV* pdev, uint32_t ch)
{
  int ret;
  //uint32_t i;
  //uint32_t acs;

  if (pdev->debug_can)
    drv_printf(pdev, "can_wait%u start wait_for_completion_interruptible_timeout\n", ch);
  ret = wait_for_completion_interruptible_timeout(&pdev->acs_completion[ch], msecs_to_jiffies(10));
  if (ret < 0)
  {
    if (pdev->debug_can || pdev->debug_wait)
      drv_printf_err(pdev, "can_wait%u wait_for_completion_interruptible_timeout error %d\n", ch, ret);
    return ret;
  }
  if (ret == 0)
  {
    if (pdev->debug_can || pdev->debug_wait)
      drv_printf_err(pdev, "can_wait%u timeout\n", ch);
    if (pdev->debug_halt_on_timeout)
      pdev->halted = 1;
    return -ETIME;
  }
/*
  if ((INL(pdev, reg_acs[ch]) & 1) != 0)
  {
    if (pdev->debug_can || pdev->debug_wait)
      drv_printf_err(pdev, "can_wait%u error\n", ch);
    return -EBUSY;
  }
*/
  if (pdev->debug_can)
    drv_printf(pdev, "can_wait%u ok\n", ch);
  return 0;
}

static int can_reg_read(TDEV* pdev, uint32_t ch, uint8_t addr, uint8_t* pdata, uint8_t nsize)
{
  int ret;

  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_read%u %02XH,%u start lock\n", ch, addr, nsize);
  ret = DRV_LOCK_CAN(pdev, ch);
  if (ret != 0)
    return ret;
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_read%u %02XH,%u got lock\n", ch, addr, nsize);
  if (pdev->halted)
  {
    ret = -EIO;
    goto toret;
  }
  reinit_completion(&pdev->acs_completion[ch]);
  OUTL(pdev, reg_acs[ch], (addr << 24) | ((nsize - 1) << 16) | 3);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_read%u %02XH,%u write ACS\n", ch, addr, nsize);
  ret = can_wait(pdev, ch);
  if (ret != 0)
    goto toret;
  bar_read(pdev, reg_buf[ch], pdata, nsize);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_read%u %02XH,%u bar_read\n", ch, addr, nsize);
toret:
  DRV_UNLOCK_CAN(pdev, ch);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_read%u %02XH,%u release lock\n", ch, addr, nsize);
  if (ret == 0 && pdev->debug_regs_can)
  {
    char s[64];
    uint32_t i;
    uint32_t slen = 0;
    for (i = 0; i < nsize; i ++)
      slen += snprintf(s + slen, sizeof(s) - slen, " %02XH", pdata[i]);
    drv_printf(pdev, "CAN%u[%02XH]=>%s\n", ch + 1, addr, s);
  }
  return ret;
}

static int can_reg_write(TDEV* pdev, uint32_t ch, uint8_t addr, uint8_t* pdata, uint8_t nsize, int autorts)
{
  int ret;

  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_write%u %02XH,%u start lock\n", ch, addr, nsize);
  ret = DRV_LOCK_CAN(pdev, ch);
  if (ret != 0)
    return ret;
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_write%u %02XH,%u got lock\n", ch, addr, nsize);
  if (pdev->halted)
  {
    ret = -EIO;
    goto toret;
  }
  bar_write(pdev, reg_buf[ch], pdata, nsize);
  reinit_completion(&pdev->acs_completion[ch]);
  OUTL(pdev, reg_acs[ch], (addr << 24) | ((nsize - 1) << 16) | (autorts ? (1 << 15) : 0) | 2);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_write%u %02XH,%u write ACS\n", ch, addr, nsize);
  ret = can_wait(pdev, ch);
toret:
  DRV_UNLOCK_CAN(pdev, ch);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_write%u %02XH,%u release lock\n", ch, addr, nsize);
  if (ret == 0 && pdev->debug_regs_can)
  {
    char s[64];
    uint32_t i;
    uint32_t slen = 0;
    for (i = 0; i < nsize; i ++)
      slen += snprintf(s + slen, sizeof(s) - slen, " %02XH", pdata[i]);
    drv_printf(pdev, "CAN%u[%02XH]<=%s\n", ch + 1, addr, s);
  }
  return ret;
}

static int can_reg_modify(TDEV* pdev, uint32_t ch, uint8_t addr, uint16_t data)
{
  int ret;

  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_modify%u %02XH,%04XH start lock\n", ch, addr, data);
  ret = DRV_LOCK_CAN(pdev, ch);
  if (ret != 0)
    return ret;
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_modify%u %02XH,%04XH got lock\n", ch, addr, data);
  if (pdev->halted)
  {
    ret = -EIO;
    goto toret;
  }
  bar_write(pdev, reg_buf[ch], (uint8_t*)&data, sizeof(data));
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_modify%u %02XH,%04XH bar_write\n", ch, addr, data);
  reinit_completion(&pdev->acs_completion[ch]);
  OUTL(pdev, reg_acs[ch], (addr << 24) | ((sizeof(data) - 1) << 16) | 5);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_modify%u %02XH,%04XH write ACS\n", ch, addr, data);
  ret = can_wait(pdev, ch);
toret:
  DRV_UNLOCK_CAN(pdev, ch);
  if (pdev->debug_can)
    drv_printf(pdev, "can_reg_modify%u %02XH,%04XH release lock\n", ch, addr, data);
  if (ret == 0 && pdev->debug_regs_can)
    drv_printf(pdev, "CAN%u[%02XH]&%02XH<=%02XH\n", ch + 1, addr, data >> 8, data & 0xFF);
  return ret;
}

static int can_rts(TDEV* pdev, uint32_t ch, uint32_t nbuf)
{
  int ret;

  if (pdev->debug_can)
    drv_printf(pdev, "can_rts%u.%u start lock\n", ch, nbuf);
  ret = DRV_LOCK_CAN(pdev, ch);
  if (ret != 0)
    return ret;
  if (pdev->debug_can)
    drv_printf(pdev, "can_rts%u.%u got lock\n", ch, nbuf);
  if (pdev->halted)
  {
    ret = -EIO;
    goto toret;
  }
  OUTL(pdev, reg_acs[ch], (can_reg_tx_buf[nbuf] << 24) | (1 << 15) | (1 << 14) | 2);
  if (pdev->debug_can)
    drv_printf(pdev, "can_rts%u.%u write ACS\n", ch, nbuf);
toret:
  DRV_UNLOCK_CAN(pdev, ch);
  if (pdev->debug_can)
    drv_printf(pdev, "can_rts%u.%u release lock\n", ch, nbuf);
  return ret;
}

static int can_reset(TDEV* pdev, uint32_t ch)
{
  int ret;
  uint32_t nbuf, filt;
  static uint8_t buf[] = { 0, 0, 0, 0 };
  DRV_LOCK_VARS();

  DRV_LOCK(pdev);
  INL_OR_OUTL(pdev, reg_ctrl[ch], 1 << 31);
  for (nbuf = 0; nbuf < CAN_BUFS; nbuf ++)
    pdev->txbuf[ch][nbuf].wrote = 0;
  DRV_UNLOCK(pdev);
  udelay(100);
  // reset RXM
  for (nbuf = 0; nbuf < nofels(can_reg_mask); nbuf ++)
  {
    ret = can_reg_modify(pdev, ch, can_reg_rx_buf[nbuf], (RXB_MODE_OFF << (8 + 5)) | ((RXB_MODE_NUM - 1) << 5));
    if (ret != 0)
      return ret;
  }
  // reset filters
  for (filt = 0; filt < nofels(can_reg_filt); filt ++)
  {
    ret = can_reg_write(pdev, ch, can_reg_filt[filt], buf, sizeof(buf), 0);
    if (ret != 0)
      return ret;
  }
  // reset masks
  for (nbuf = 0; nbuf < nofels(can_reg_mask); nbuf ++)
  {
    ret = can_reg_write(pdev, ch, can_reg_mask[nbuf], buf, sizeof(buf), 0);
    if (ret != 0)
      return ret;
  }
  // enable DMA
  ret = can_reg_modify(pdev, ch, BFPCTRL, 0x0F0F);
  if (ret != 0)
    return ret;
  // enable interrupts
  ret = can_reg_modify(pdev, ch, CANINTE, 0x1C1C);
  if (ret != 0)
    return ret;
  // enable triggers
  ret = can_reg_modify(pdev, ch, TXRTSCTRL, 0x0707);
  if (ret != 0)
    return ret;
  return ret;
}

static int drv_reset(TDEV* pdev)
{
  int ret;
  uint32_t ch, nbuf;
  DRV_LOCK_VARS();

  DRV_LOCK(pdev);
  drv_uninit_ints(pdev);
  INL(pdev, OFFSET_INTERRUPT_REG_TTCAN); // clear interrupt
  INL_AND_OR_OUTL(pdev, OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN, ~(1 << 0), (1 << 2) | (1 << 1)); // disable DMA
  udelay(1000);
  for (ch = 0; ch < 2; ch ++)
  {
    OUTL(pdev, reg_dma_index[ch], 0);
    OUTL(pdev, reg_dma_rd_index[ch], 0);
    pdev->dma_wrptr[ch] = pdev->dma_rdptr[ch] = 0;
    INL_AND_OR_OUTL(pdev, reg_timer_ctrl[ch], ~((0xFF << 8) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1)), 1);
    INL_AND_OR_OUTL(pdev, reg_trig_ctrl[ch],
                    ~((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12) | (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8) |
                      (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0)),
                    (2 << 6) | (2 << 4) | (2 << 2) | (2 << 0));
    for (nbuf = 0; nbuf < CAN_BUFS; nbuf ++)
    {
      OUTL(pdev, reg_trig[ch][nbuf], 0);
      OUTL(pdev, reg_trig_epoch[ch][nbuf], 0);
    }
    OUTL(pdev, reg_timer_trsh[ch], 0);
    OUTL(pdev, reg_timer_ceed[ch], 0);
    OUTL(pdev, reg_epoch_ceed[ch], 0);
    OUTL(pdev, reg_timeout_absolute[ch], 0);
    OUTL(pdev, reg_timeout_interval[ch], 0);
    pdev->dma_wr[ch] = (uint32_t*)(pdev->dma_vaddr + ch * TTCAN_CH_DMA_SIZE);
    pdev->dma_rd[ch] = (uint32_t*)(pdev->dma_wr[ch]);
  }
  memset(pdev->dma_vaddr, 0, pdev->dma_size);
  INL_OR_OUTL(pdev, OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN, (1 << 2) | (1 << 1));
  drv_init_ints(pdev);
  DRV_UNLOCK(pdev);
  for (ch = 0; ch < 2; ch ++)
  {
    ret = can_reset(pdev, ch);
    if (ret != 0)
      return ret;
  }
  return 0;
}

static int drv_probe(struct pci_dev* pdevice, const struct pci_device_id* pid)
{
  int ret;
  TDEV* pdev;
  uint32_t i, j;
  uint16_t vendorid = pdevice->vendor;
  uint16_t deviceid = pdevice->device;
  uint8_t revisionid = pdevice->revision;
  dev_t devt;
  uint8_t link_width, link_speed;
  uint32_t devtype;
  char devtypestr[16];

  if (vendorid != NM_TTCAN_VENDOR_ID ||
      deviceid != NM_TTCAN_DEVICE_ID ||
      (revisionid != NM_TTCAN_REVISION_ID_FW_01 &&
       revisionid != NM_TTCAN_REVISION_ID_FW_02 &&
       revisionid != NM_TTCAN_REVISION_ID_FW_03 &&
       revisionid != 0x13)
       ) {
      printk("NMTTCAN: drv_probe vendorid = %x, deviceid = %x,  revisionid = %x\n", vendorid, deviceid, revisionid);
    return -ENXIO;
  }

  ret = pci_enable_device_mem(pdevice);
  if (ret != 0)
  {
    pr_err("%s pci_enable_device_mem error %d\n", drv_name, ret);
    return ret;
  }

  pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
  if (pdev == NULL)
  {
    pr_err("%s probe: can't allocate %llu bytes for device\n", drv_name, (unsigned long long)sizeof(*pdev));
    ret = -ENOMEM;
    goto err_alloc;
  }

  pdev->debug = drv_debug;
  pdev->debug_regs = drv_debug_regs;
  pdev->debug_regs_can = drv_debug_regs_can;
  pdev->debug_dma = drv_debug_dma;
  pdev->debug_intr = drv_debug_intr;
  pdev->debug_can = drv_debug_can;
  pdev->debug_send = drv_debug_send;
  pdev->debug_wait = drv_debug_wait;

  pdev->pdevice = pdevice;
  for (i = 0; i < MAX_NUM_BOARDS; i ++)
  {
    if (pdevs[i] == NULL)
    {
      pdevs[i] = pdev;
      pdev->unit = i;
      break;
    }
  }
  if (i == MAX_NUM_BOARDS)
  {
    pr_err("%s probe: too many devices in system\n", drv_name);
    ret = -ENXIO;
    goto err_unit;
  }

  SNPRINTF(pdev->devname, "%s%u", "ttcan_dev_", pdev->unit);

  pdev->vendorid = vendorid;
  pdev->deviceid = deviceid;
  pdev->revisionid = revisionid;

  spin_lock_init(&pdev->lock);
  for (i = 0; i < 2; i ++)
  {
    spin_lock_init(&pdev->lock_reader[i]);
    sema_init(&pdev->sem_can[i], 1);
    for (j = 0; j < CAN_BUFS; j ++)
      sema_init(&pdev->sem_can_buf[i][j], 1);
    sema_init(&pdev->sem_read[i], 1);
    init_completion(&pdev->acs_completion[i]);
    for (j = 0; j < CAN_BUFS; j ++)
      init_completion(&pdev->xmtd_completion[i][j]);
    init_completion(&pdev->timer_completion[i]);
    init_completion(&pdev->read_completion[i]);
  }
  mutex_init(&pdev->mtxR);
  mutex_init(&pdev->mtxT1);
  mutex_init(&pdev->mtxT2);

  drv_printf(pdev, "internal data size %llu bytes @%p\n", (unsigned long long)sizeof(*pdev), pdev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
  if ((ret = pci_set_dma_mask(pdevice, DMA_BIT_MASK(64))) == 0)
  {
    ret = pci_set_consistent_dma_mask(pdevice, DMA_BIT_MASK(64));
    if (ret != 0)
    {
      drv_printf_err(pdev, "pci_set_consistent_dma_mask(64) error %d\n", ret);
      goto err_pci_reg;
    }
  }
  else if ((ret = pci_set_dma_mask(pdevice, DMA_BIT_MASK(32))) == 0)
  {
    ret = pci_set_consistent_dma_mask(pdevice, DMA_BIT_MASK(32));
    if (ret != 0)
    {
      drv_printf_err(pdev, "pci_set_consistent_dma_mask(32) error %d\n", ret);
      goto err_pci_reg;
    }
  }
  else
  {
    drv_printf_err(pdev, "no usable DMA configuration\n");
    goto err_pci_reg;
  }
#endif
  ret = pci_request_selected_regions(pdevice, pci_select_bars(pdevice, IORESOURCE_MEM), drv_name);
  if (ret != 0)
  {
    drv_printf_err(pdev, "pci_request_selected_regions error %d\n", ret);
    goto err_pci_reg;
  }

  pci_set_master(pdevice);
  pci_set_drvdata(pdevice, pdev);

  pdev->pbar = pci_iomap(pdevice, 0, 0);
  if (pdev->pbar == NULL)
  {
    drv_printf_err(pdev, "could not map BAR0 memory\n");
    ret = -ENXIO;
    goto err_pci_map;
  }
  pdev->barlen = pci_resource_len(pdevice, 0);

  devtype = (INL(pdev, 0x1034) >> 16) & 0xFF;
  if (devtype < nofels(drv_devtype_str))
    STRNCPY(devtypestr, drv_devtype_str[devtype]);
  else
    SNPRINTF(devtypestr, "UNKNOWN %02XH", devtype);
  drv_printf(pdev, "Type: '%s' Revision ID: %02XH\n", devtypestr, revisionid);

  drv_printf(pdev, "BAR0 len %llu phys@%llXH virt@%p\n", (unsigned long long)pdev->barlen, (unsigned long long)pci_resource_start(pdevice, 0), pdev->pbar);

  pdev->dma_size = TTCAN_DMA_SIZE;
  pdev->dma_vaddr = (uint8_t*)dma_alloc_coherent(&pdevice->dev, pdev->dma_size, &pdev->dma_paddr, GFP_KERNEL);
  if (pdev->dma_vaddr == NULL)
  {
    drv_printf_err(pdev, "could not allocate %lu bytes for data buffer\n", (unsigned long)pdev->dma_size);
    ret = -ENOMEM;
    goto err_data_buf;
  }
  memset(pdev->dma_vaddr, 0, pdev->dma_size);
  for (i = 0; i < 2; i ++) {
    pdev->ch_dma_vaddr[i] = pdev->dma_vaddr + i * TTCAN_CH_DMA_SIZE;
    pdev->dma_wr[i] = (uint32_t*)(pdev->dma_vaddr + i * TTCAN_CH_DMA_SIZE);
    pdev->dma_rd[i] = (uint32_t*)(pdev->dma_wr[i]);
  }

  drv_printf(pdev, "DMA_DATA_BASE phys@%llXH virt@%p\n", (unsigned long long)pdev->dma_paddr, pdev->dma_vaddr);

  OUTL(pdev, OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN, (uint32_t)pdev->dma_paddr | (1 << 2) | (1 << 1));
  OUTL(pdev, OFFSET_DMA_DATA_BASE_HI_REG_mPCIe_CAN, (uint32_t)((uint64_t)pdev->dma_paddr >> 32));

  ret = pci_enable_msi(pdevice);
  if (ret != 0)
  {
    drv_printf_err(pdev, "pci_enable_msi error %d\n", ret);
    goto err_msi;
  }

  ret = request_irq(pdevice->irq, drv_intr_handler, IRQF_SHARED, pdev->devname, pdev);
  if (ret != 0)
  {
    drv_printf_err(pdev, "request_irq error %d\n", ret);
    goto err_irq;
  }

  drv_printf(pdev, "IRQ %u\n", pdevice->irq);

  ret = sysfs_create_group(&pdevice->dev.kobj, &drv_attr_group);
  if (ret != 0)
  {
    drv_printf_err(pdev, "sysfs_create_group error %d\n", ret);
    goto err_sysfs_group;
  }

  devt = MKDEV(drv_cdev_major, pdev->unit);

  cdev_init(&pdev->cdev, &drv_fops);
  pdev->cdev.owner = THIS_MODULE;
  pdev->cdev.ops = &drv_fops;
  ret = cdev_add(&pdev->cdev, devt, 1);
  if (ret != 0)
  {
    drv_printf_err(pdev, "cdev_add error %d\n", ret);
    goto err_cdev_add;
  }

  pdev->pcdev_device = device_create(drv_cdev_class, NULL, devt, NULL, pdev->devname);
  if (IS_ERR(pdev->pcdev_device))
  {
    ret = PTR_ERR(pdev->pcdev_device);
    drv_printf_err(pdev, "device_create error %d\n", ret);
    goto err_device_create;
  }

  ret = sysfs_create_link(&pdev->pcdev_device->kobj, &pdevice->dev.kobj, "device");
  if (ret != 0)
  {
    drv_printf_err(pdev, "sysfs_create_link error %d\n", ret);
    goto err_sysfs_link;
  }

  pci_express_get_negotiated_link_status(pdevice, &link_width, &link_speed);
  drv_printf(pdev, "PCI Express Negotiated Link Width: x%u Speed: x%u\n", link_width, link_speed);

  //drv_init(pdev);
  drv_init_ints(pdev);

  //drv_init_timeouts(pdev);

  drv_reset(pdev);

  pdev->flags |= DRV_INITIALIZED;

  return 0;

err_sysfs_link:
  device_destroy(drv_cdev_class, devt);
err_device_create:
  cdev_del(&pdev->cdev);
err_cdev_add:
  sysfs_remove_group(&pdevice->dev.kobj, &drv_attr_group);
err_sysfs_group:
  free_irq(pdevice->irq, pdev);
err_irq:
  pci_disable_msi(pdevice);
err_msi:
  dma_free_coherent(&pdevice->dev, pdev->dma_size, pdev->dma_vaddr, pdev->dma_paddr);
  for (i = 0; i < 2; i ++)
    pdev->ch_dma_vaddr[i] = NULL;
  pdev->dma_vaddr = NULL;
  pdev->dma_paddr = 0;
  pdev->dma_size = 0;
err_data_buf:
  pci_iounmap(pdevice, pdev->pbar);
err_pci_map:
  pci_release_selected_regions(pdevice, pci_select_bars(pdevice, IORESOURCE_MEM));
err_pci_reg:
  pdevs[pdev->unit] = NULL;
err_unit:
  kfree(pdev);
err_alloc:
  pci_disable_device(pdevice);

  return ret;
}

static void drv_remove(struct pci_dev* pdevice)
{
  TDEV* pdev = (TDEV*)pci_get_drvdata(pdevice);
  uint32_t i;
  dev_t devt = MKDEV(drv_cdev_major, pdev->unit);

  drv_reset(pdev);

  //drv_release(pdev);

  drv_uninit_ints(pdev);

  sysfs_remove_link(&pdev->pcdev_device->kobj, "device");
  device_destroy(drv_cdev_class, devt);
  cdev_del(&pdev->cdev);
  sysfs_remove_group(&pdevice->dev.kobj, &drv_attr_group);

  free_irq(pdevice->irq, pdev);
  pci_disable_msi(pdevice);

  dma_free_coherent(&pdevice->dev, pdev->dma_size, pdev->dma_vaddr, pdev->dma_paddr);
  for (i = 0; i < 2; i ++)
    pdev->ch_dma_vaddr[i] = NULL;
  pdev->dma_vaddr = NULL;
  pdev->dma_paddr = 0;
  pdev->dma_size = 0;

  mutex_destroy(&pdev->mtxR);
  mutex_destroy(&pdev->mtxT1);
  mutex_destroy(&pdev->mtxT2);

  pci_iounmap(pdevice, pdev->pbar);
  pci_release_selected_regions(pdevice, pci_select_bars(pdevice, IORESOURCE_MEM));

  pdev->flags = 0;
  pdevs[pdev->unit] = NULL;
  kfree(pdev);
  pci_disable_device(pdevice);
}

static void drv_shutdown(struct pci_dev* pdevice)
{
  TDEV* pdev = (TDEV*)pci_get_drvdata(pdevice);

  drv_uninit_ints(pdev);

  OUTL(pdev, 0x1024, 0);
}

static int drv_open(struct inode* pinode, struct file* pfile)
{
  TDEV* pdev;
  u_int unit;
  DRV_LOCK_VARS();

  unit = iminor(pinode);
  pdev = pdevs[unit];
  if (pdev == NULL)
    return -ENXIO;

  if ((pdev->flags & DRV_INITIALIZED) == 0)
  {
    drv_printf_err(pdev, "open: device is not initialized\n");
    return -ENXIO;
  }

  DRV_LOCK(pdev);
  pci_dev_get(pdev->pdevice);
  pdev->busy ++;
  DRV_UNLOCK(pdev);

  pfile->private_data = pdev;

  return 0;
}

static int drv_close(struct inode* pinode, struct file* pfile)
{
  TDEV* pdev;
  DRV_LOCK_VARS();

  pdev = (TDEV*)pfile->private_data;
  if (pdev == NULL)
    return -ENXIO;

  DRV_LOCK(pdev);
  pci_dev_put(pdev->pdevice);
  pdev->busy --;
  DRV_UNLOCK(pdev);

  return 0;
}

static ssize_t drv_read(struct file* pfile, char __user* pbuf, size_t buflen, loff_t* offset)
{
#if 0
  TDEV* pdev;
  ssize_t size;
  long ret;
  DRV_Snapshot* pshot;
  DRV_LOCK_VARS();

  pdev = (TDEV*)pfile->private_data;
  size = 0;
  if (pdev == NULL)
    return -ENXIO;

  if (buflen == 0)
    return 0;

  if ((pdev->flags & OM_READSNAPSHOT) == 0)
    return -ENXIO;

  if (buflen < sizeof(*pshot))
  {
    DRV_LOCK(pdev);
    pdev->flags &= ~DRV_READSNAPSHOT;
    DRV_UNLOCK(pdev);
    return -EINVAL;
  }

  pshot = (DRV_Snapshot*)vmalloc(sizeof(*pshot));
  if (pshot == NULL)
  {
    DRV_LOCK(pdev);
    pdev->flags &= ~DRV_READSNAPSHOT;
    DRV_UNLOCK(pdev);
    return -ENOMEM;
  }

  DRV_LOCK(pdev);
  bcopy(&pdev->m_Snapshot, pshot, sizeof(*pshot));
  pdev->flags &= ~(DRV_READSNAPSHOT | DRV_GOTSNAPSHOT);
  DRV_UNLOCK(pdev);

  ret = copy_to_user(pbuf, (caddr_t)pshot, sizeof(*pshot));
  if (ret != 0)
    return -EFAULT;
  size += sizeof(*pshot);

  vfree(pshot);

  return size;
#else
  return -EPERM;
#endif
}

static ssize_t drv_write(struct file* pfile, const char __user* pbuf, size_t buflen, loff_t* offset)
{
  return -EPERM;
}

static int drv_mmap(struct file* pfile, struct vm_area_struct* pvma)
{
  TDEV* pdev;
  void* pbuf;
  unsigned long size, offset;

  pdev = (TDEV*)pfile->private_data;
  if (pdev == NULL)
    return -ENXIO;

  if (pfile->f_mode & FMODE_EXEC)
    return -EPERM;
  if (pvma->vm_start > pvma->vm_end)
    return -EPERM;
  size = pvma->vm_end - pvma->vm_start;
  offset = pvma->vm_pgoff << PAGE_SHIFT;
  if (size > TTCAN_DMA_SIZE || offset + size > TTCAN_DMA_SIZE)
    return -EPERM;
  pbuf = pdev->dma_vaddr;
  pvma->vm_flags |= VM_IO;
#ifdef VM_RESERVED
  pvma->vm_flags |= VM_RESERVED;
#endif
#ifdef VM_DONTDUMP
  pvma->vm_flags |= VM_DONTDUMP;
#endif
  if (remap_pfn_range(pvma, pvma->vm_start, __pa(pbuf + offset) >> PAGE_SHIFT, size, pvma->vm_page_prot))
    return -EAGAIN;
  return 0;
}

static irqreturn_t drv_intr_handler(int irq, void* arg)
{
  TDEV* pdev = (TDEV*)arg;
  uint32_t ri;
  uint32_t ch, nbuf;
//  uint32_t can1ctrl, can2ctrl;

  static const uint32_t int_can_acs_mask[2] = { INT_CAN_ACS1_TTCAN, INT_CAN_ACS2_TTCAN };
  static const uint32_t int_msg_xmtd_mask[2][CAN_BUFS] =
  {
    { INT_MSG_XMTD10_TTCAN, INT_MSG_XMTD11_TTCAN, INT_MSG_XMTD12_TTCAN },
    { INT_MSG_XMTD20_TTCAN, INT_MSG_XMTD21_TTCAN, INT_MSG_XMTD22_TTCAN }
  };
  static const uint32_t int_timer_mask[2] = { INT_TIM_CAN1_TTCAN, INT_TIM_CAN2_TTCAN };
  static const uint32_t int_abs_timer_mask[2] = { INT_TIM_ABS1_TTCAN, INT_TIM_ABS2_TTCAN };
  static const uint32_t int_itv_timer_mask[2] = { INT_TIM_ITV1_TTCAN, INT_TIM_ITV2_TTCAN };
  static const uint32_t int_msg_rcvd_mask[2] = { INT_MSG_RCVD1_TTCAN, INT_MSG_RCVD2_TTCAN };

  ri = INL(pdev, OFFSET_INTERRUPT_REG_TTCAN);

  if (pdev->debug_intr)
    drv_printf(pdev, "INT=%08XH\n", ri);

  ri &= pdev->int_mask;
  if (ri == 0)
    return IRQ_NONE;

//  can1ctrl = ioread32(pdev->pbar + OFFSET_CAN1_CTRL_REG_PCI_mPCIe_CAN);
//  can2ctrl = ioread32(pdev->pbar + OFFSET_CAN2_CTRL_REG_PCI_mPCIe_CAN);

//  if ((can1ctrl& (1<<5)) > 0)
//      return IRQ_HANDLED;
//  if ((can2ctrl& (1<<5)) > 0)
//      return IRQ_HANDLED;

  if (pdev->debug_intr)
    for (ch = 0; ch < 2; ch ++)
      if ((ri & int_timer_mask[ch]) != 0)
      {
        uint32_t timer = INL(pdev, reg_timer[ch]);
        uint32_t epoch = INL(pdev, reg_timer_epoch[ch]);
        drv_printf(pdev, "INT timer%u=%u.%05u.%05u\n", ch, epoch, timer >> 16, (timer >> 2) & 0x3FFF);
      }

  for (ch = 0; ch < 2; ch ++)
  {
    uint32_t wrptr, rdptr, datasize;
    int dma_index_error;
    DRV_LOCK_READER_VARS();

    if ((ri & int_can_acs_mask[ch]) != 0)
      complete(&pdev->acs_completion[ch]);

    for (nbuf = 0; nbuf < CAN_BUFS; nbuf ++)
      if ((ri & int_msg_xmtd_mask[ch][nbuf]) != 0)
        complete(&pdev->xmtd_completion[ch][nbuf]);

    if ((ri & int_timer_mask[ch]) != 0)
      complete_all(&pdev->timer_completion[ch]);

    if ((ri & int_abs_timer_mask[ch]) != 0 ||
        (ri & int_itv_timer_mask[ch]) != 0)
    {
      wrptr = INL(pdev, reg_dma_index[ch]);
      dma_index_error = (wrptr / TTCAN_CH_DMA_SIZE != 0 || wrptr % TTCAN_DMA_SLOT_SIZE != 0) ? 1 : 0;
      if (pdev->debug_dma)
        drv_printf(pdev, "INT DMA_INDEX%u=%08XH%s\n", ch, wrptr, dma_index_error ? " - ERROR" : "");
      if (dma_index_error == 0)
      {
        pdev->dma_wrptr[ch] = wrptr;
        DRV_LOCK_READER(pdev, ch);
        if (pdev->reader_sleeping[ch] != 0)
        {
          DRV_UNLOCK_READER(pdev, ch);
          if (pdev->debug_dma)
            drv_printf(pdev, "INT%u completion\n", ch);
          complete(&pdev->read_completion[ch]);
        }
        else
          DRV_UNLOCK_READER(pdev, ch);
      }
    }

    if ((ri & int_msg_rcvd_mask[ch]) != 0)
    {
      wrptr = INL(pdev, reg_dma_index[ch]);
      rdptr = pdev->dma_rdptr[ch];
      dma_index_error = (wrptr / TTCAN_CH_DMA_SIZE != 0 || wrptr % TTCAN_DMA_SLOT_SIZE != 0) ? 1 : 0;
      if (pdev->debug_dma)
        drv_printf(pdev, "INT DMA_INDEX%u=%08XH%s\n", ch, wrptr, dma_index_error ? " - ERROR" : "");
      if (dma_index_error == 0)
      {
        pdev->dma_wrptr[ch] = wrptr;
        datasize = (wrptr + TTCAN_CH_DMA_SIZE - rdptr) % TTCAN_CH_DMA_SIZE;
        if (datasize > 0 && pdev->debug_dma)
          drv_printf(pdev, "INT%u WRPTR=%u RDPTR=%u DIFF=%u blocks\n", ch,
                      wrptr / TTCAN_DMA_SLOT_SIZE, rdptr / TTCAN_DMA_SLOT_SIZE, datasize / TTCAN_DMA_SLOT_SIZE);
        DRV_LOCK_READER(pdev, ch);
        if (pdev->reader_sleeping[ch] != 0 &&
            (datasize >= pdev->reader_size[ch] || datasize >= TTCAN_CH_DMA_SIZE / 2))
        {
          DRV_UNLOCK_READER(pdev, ch);
          if (pdev->debug_dma)
            drv_printf(pdev, "INT%u completion\n", ch);
          complete(&pdev->read_completion[ch]);
        }
        else
          DRV_UNLOCK_READER(pdev, ch);
      }
    }
  }

  return IRQ_HANDLED;
}

static int drv_wr_reg(TDEV* pdev, unsigned long addr)
{
    SADDR_DATA_MAIN_TTCAN saddr;

    if (copy_from_user((void*)&saddr.daddr, (void*)(addr + ((uint8_t*)&saddr.daddr - (uint8_t*)&saddr)), sizeof(saddr.daddr)) != 0)
        return -EFAULT;
    if (saddr.daddr % sizeof(uint32_t) != 0 ||
            saddr.daddr + sizeof(uint32_t) > pdev->barlen)
        return -EINVAL;
    if (copy_from_user((void*)&saddr.data, (void*)(addr + ((uint8_t*)&saddr.data - (uint8_t*)&saddr)), sizeof(saddr.data)) != 0)
        return -EFAULT;
    OUTL(pdev, saddr.daddr, saddr.data);
    if (pdev->debug_regs)
        drv_printf(pdev, "REG[%04XH]<=%08XH\n", saddr.daddr, saddr.data);
    return 0;
}

static int drv_rd_reg(TDEV* pdev, unsigned long addr)
{
  SADDR_DATA_MAIN_TTCAN saddr;

  if (copy_from_user((void*)&saddr.daddr, (void*)(addr + ((uint8_t*)&saddr.daddr - (uint8_t*)&saddr)), sizeof(saddr.daddr)) != 0)
    return -EFAULT;
  if (saddr.daddr % sizeof(uint32_t) != 0 ||
      saddr.daddr + sizeof(uint32_t) > pdev->barlen)
    return -EINVAL;
  saddr.data = INL(pdev, saddr.daddr);
  if (copy_to_user((void*)(addr + ((uint8_t*)&saddr.data - (uint8_t*)&saddr)), (void*)&saddr.data, sizeof(saddr.data)) != 0)
    return -EFAULT;
  if (pdev->debug_regs)
    drv_printf(pdev, "REG[%04XH]=>%08XH\n", saddr.daddr, saddr.data);
  return 0;
}

static int drv_wr_canreg(TDEV* pdev, unsigned long addr)
{
  SADDR_DATA_TTCAN saddr;

  if (copy_from_user((void*)&saddr, (void*)addr, sizeof(saddr)) != 0)
    return -EFAULT;
  if ((saddr.channel != 1 && saddr.channel != 2) ||
      saddr.daddr >= CAN_REGS)
    return -EINVAL;
  return can_reg_write(pdev, saddr.channel - 1, saddr.daddr, &saddr.data, 1, 0);
}

static int drv_rd_canreg(TDEV* pdev, unsigned long addr)
{
  int ret;
  SADDR_DATA_TTCAN saddr;

  if (copy_from_user((void*)&saddr, (void*)addr, sizeof(saddr)) != 0)
    return -EFAULT;
  if ((saddr.channel != 1 && saddr.channel != 2) ||
      saddr.daddr >= CAN_REGS)
    return -EINVAL;
  ret = can_reg_read(pdev, saddr.channel - 1, saddr.daddr, &saddr.data, 1);
  if (ret != 0)
    return ret;
  if (copy_to_user((void*)(addr + ((uint8_t*)&saddr.data - (uint8_t*)&saddr)), (void*)&saddr.data, sizeof(saddr.data)) != 0)
    return -EFAULT;
  return 0;
}

static int drv_modify_canreg(TDEV* pdev, unsigned long addr)
{
  SADDR_DATA_MODIFY_TTCAN saddr;

  if (copy_from_user((void*)&saddr, (void*)addr, sizeof(saddr)) != 0)
    return -EFAULT;
  if ((saddr.channel != 1 && saddr.channel != 2) ||
      saddr.daddr >= CAN_REGS)
    return -EINVAL;
  return can_reg_modify(pdev, saddr.channel - 1, saddr.daddr, saddr.data);
}

static int drv_wr_canregs(TDEV* pdev, unsigned long addr)
{
  SADDR_DATAS_TTCAN saddr;

  if (copy_from_user((void*)&saddr, (void*)addr, sizeof(saddr)) != 0)
    return -EFAULT;
  if ((saddr.channel != 1 && saddr.channel != 2) ||
      saddr.addr + saddr.nsize > CAN_REGS ||
      saddr.nsize == 0)
    return -EINVAL;
  return can_reg_write(pdev, saddr.channel - 1, saddr.addr, saddr.data, saddr.nsize, 0);
}

static int drv_rd_canregs(TDEV* pdev, unsigned long addr)
{
  int ret;
  SADDR_DATAS_TTCAN saddr;

  if (copy_from_user((void*)&saddr, (void*)addr, sizeof(saddr)) != 0)
    return -EFAULT;
  if ((saddr.channel != 1 && saddr.channel != 2) ||
      saddr.addr + saddr.nsize > CAN_REGS ||
      saddr.nsize == 0)
    return -EINVAL;
  ret = can_reg_read(pdev, saddr.channel - 1, saddr.addr, saddr.data, saddr.nsize);
  if (ret != 0)
    return ret;
  if (copy_to_user((void*)(addr + ((uint8_t*)&saddr.data - (uint8_t*)&saddr)), (void*)&saddr.data, saddr.nsize) != 0)
    return -EFAULT;
  return 0;
}

static int drv_enable_dma(TDEV* pdev, unsigned long addr)
{
  int ret;
  uint32_t ch;
  DRV_LOCK_VARS();

  for (ch = 0; ch < 2; ch ++)
  {
    ret = can_reg_modify(pdev, ch, BFPCTRL, 0x0F0F);
    if (ret != 0)
      return ret;
  }
  DRV_LOCK(pdev);
  INL_OR_OUTL(pdev, OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN, (1 << 2) | (1 << 1) | (1 << 0));
//  INL_OR_OUTL(pdev, OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN, (0 << 2) | (1 << 1) | (1 << 0));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_disable_dma(TDEV* pdev, unsigned long addr)
{
  int ret;
  uint32_t ch;
  DRV_LOCK_VARS();

  for (ch = 0; ch < 2; ch ++)
  {
    ret = can_reg_modify(pdev, ch, BFPCTRL, 0x000F);
    if (ret != 0)
      return ret;
  }
  DRV_LOCK(pdev);
  INL_AND_OR_OUTL(pdev, OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN, ~(1 << 0), (1 << 2) | (1 << 1));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_rd_ch_raw_dma(TDEV* pdev, unsigned long addr)
{
  int ret;
  uint32_t nchannel, nblock_read, nblock, ch, timeout, wrptr, rdptr, blk;
  DMA_STR_TTCAN* pdma = NULL;
  DRV_LOCK_READER_VARS();

  if (copy_from_user((void*)&nblock_read, (void*)(addr + ((uint8_t*)&pdma->number_block - (uint8_t*)pdma)), sizeof(nblock_read)) != 0)
    return -EFAULT;
  if (nblock_read == 0 || nblock_read > sizeof(pdma->buf) / TTCAN_DMA_SLOT_SIZE)
    return -EINVAL;
  if (copy_from_user((void*)&nchannel, (void*)(addr + ((uint8_t*)&pdma->number_channel - (uint8_t*)pdma)), sizeof(nchannel)) != 0)
    return -EFAULT;
  if (nchannel != 1 && nchannel != 2)
    return -EINVAL;
  if (copy_from_user((void*)&timeout, (void*)(addr + ((uint8_t*)&pdma->timeout - (uint8_t*)pdma)), sizeof(timeout)) != 0)
    return -EFAULT;
  ch = nchannel - 1;
  ret = DRV_LOCK_READ(pdev, ch);
  if (ret != 0)
    return ret;
  wrptr = pdev->dma_wrptr[ch];
  rdptr = pdev->dma_rdptr[ch];
  nblock = ((wrptr + TTCAN_CH_DMA_SIZE - rdptr) % TTCAN_CH_DMA_SIZE) / TTCAN_DMA_SLOT_SIZE;
  if (timeout > 0 && nblock < TTCAN_CH_DMA_SIZE / 2 / TTCAN_DMA_SLOT_SIZE && nblock < nblock_read)
  {
    DRV_LOCK_READER(pdev, ch);
    pdev->reader_size[ch] = nblock_read * TTCAN_DMA_SLOT_SIZE;
    pdev->reader_sleeping[ch] = 1;
    reinit_completion(&pdev->read_completion[ch]);
    DRV_UNLOCK_READER(pdev, ch);
    ret = wait_for_completion_interruptible_timeout(&pdev->read_completion[ch], msecs_to_jiffies(timeout));
    DRV_LOCK_READER(pdev, ch);
    pdev->reader_sleeping[ch] = 0;
    pdev->reader_size[ch] = 0;
    DRV_UNLOCK_READER(pdev, ch);
    if (ret < 0)
    {
      DRV_UNLOCK_READ(pdev, ch);
      if (pdev->debug_wait)
        drv_printf_err(pdev, "rd_ch_raw_dma%u wait_for_completion_interruptible_timeout error %d\n", ch, ret);
      return ret;
    }
    if (ret == 0)
    {
      if (pdev->debug_wait)
        drv_printf_err(pdev, "rd_ch_raw_dma%u timeout\n", ch);
    }
    wrptr = pdev->dma_wrptr[ch];
    rdptr = pdev->dma_rdptr[ch];
    nblock = ((wrptr + TTCAN_CH_DMA_SIZE - rdptr) % TTCAN_CH_DMA_SIZE) / TTCAN_DMA_SLOT_SIZE;
  }
  if (nblock > 0)
  {
    if (pdev->debug_dma)
      drv_printf(pdev, "rd_ch_raw_dma%u WRPTR=%u RDPTR=%u DIFF=%u blocks\n", ch,
                  wrptr / TTCAN_DMA_SLOT_SIZE, rdptr / TTCAN_DMA_SLOT_SIZE, nblock);
    if (nblock > nblock_read)
      nblock = nblock_read;
    for (blk = 0; blk < nblock; blk ++)
    {
      if (copy_to_user((void*)(addr + ((uint8_t*)&pdma->buf[blk] - (uint8_t*)pdma)), pdev->ch_dma_vaddr[ch] + rdptr, sizeof(pdma->buf[blk])) != 0)
      {
        DRV_UNLOCK_READ(pdev, ch);
        return -EFAULT;
      }
      rdptr = (rdptr + TTCAN_DMA_SLOT_SIZE) % TTCAN_CH_DMA_SIZE;
    }
    pdev->dma_rdptr[ch] = rdptr;
    OUTL(pdev, reg_dma_rd_index[ch], rdptr);
  }
  DRV_UNLOCK_READ(pdev, ch);
  if (copy_to_user((void*)(addr + ((uint8_t*)&pdma->number_block - (uint8_t*)pdma)), (void*)&nblock, sizeof(nblock)) != 0)
    return -EFAULT;
  if (pdev->debug_dma && nblock > 0)
    drv_printf(pdev, "rd_ch_raw_dma%u read %u blocks\n", ch, nblock);
  return 0;
}

static int drv_set_mode(TDEV* pdev, unsigned long addr)
{
  CONF_TTCAN conf;

  if (copy_from_user((void*)&conf, (void*)addr, sizeof(conf)) != 0)
    return -EFAULT;
  if (conf.channel != 1 && conf.channel != 2)
    return -EINVAL;
  switch (conf.mode)
  {
    case CAN_WORK :
    case CAN_SLEEP :
    case CAN_LOOP :
    case CAN_MON :
    case CAN_CONF :
      break;
    default :
      return -EINVAL;
  }
  return can_reg_modify(pdev, conf.channel - 1, CAN_CTRL, (conf.mode << (8 + 5)) | (7 << 5));
}

static int drv_get_mode(TDEV* pdev, unsigned long addr)
{
  int ret;
  CONF_TTCAN conf;
  uint8_t can_stat;

  if (copy_from_user((void*)&conf.channel, (void*)(addr + ((uint8_t*)&conf.channel - (uint8_t*)&conf)), sizeof(conf.channel)) != 0)
    return -EFAULT;
  if (conf.channel != 1 && conf.channel != 2)
    return -EINVAL;
  ret = can_reg_read(pdev, conf.channel - 1, CAN_STAT, &can_stat, 1);
  if (ret != 0)
    return ret;
  conf.mode = can_stat >> 5;
  if (copy_to_user((void*)(addr + ((uint8_t*)&conf.mode - (uint8_t*)&conf)), (void*)&conf.mode, sizeof(conf.mode)) != 0)
    return -EFAULT;
  return 0;
}

static int drv_set_oneshot_mode(TDEV* pdev, unsigned long addr)
{
  CONF_TTCAN conf;

  if (copy_from_user((void*)&conf, (void*)addr, sizeof(conf)) != 0)
    return -EFAULT;
  if (conf.channel != 1 && conf.channel != 2)
    return -EINVAL;
  return can_reg_modify(pdev, conf.channel - 1, CAN_CTRL, (((conf.mode != 0) ? 0x08 : 0x00) << 8) | 0x08);
}

static int drv_set_speed(TDEV* pdev, unsigned long addr)
{
  int ret;
  SPEED_TTCAN speed;
  static uint8_t can_cnf_tbl[4][3] =
  {
    { 0x04, 0xB1, 0x85 },
    { 0x03, 0xD1, 0x83 },
    { 0x01, 0xD1, 0x83 },
    { 0x00, 0xD1, 0x83 }
  };
  uint8_t* pcnf;

  if (copy_from_user((void*)&speed, (void*)addr, sizeof(speed)) != 0)
    return -EFAULT;
  if (speed.channel != 1 && speed.channel != 2)
    return -EINVAL;

  switch (speed.speed)
  {
    case WORK_SPEED_125 :
      pcnf = can_cnf_tbl[0];
      break;
    case WORK_SPEED_250 :
      pcnf = can_cnf_tbl[1];
      break;
    case WORK_SPEED_500 :
      pcnf = can_cnf_tbl[2];
      break;
    case WORK_SPEED_1000 :
      pcnf = can_cnf_tbl[3];
      break;
    default :
      return -EINVAL;
  }
  ret = can_reg_write(pdev, speed.channel - 1, CAN_CNF1, &pcnf[0], 1, 0);
  if (ret != 0)
    return ret;
  ret = can_reg_write(pdev, speed.channel - 1, CAN_CNF2, &pcnf[1], 1, 0);
  if (ret != 0)
    return ret;
  ret = can_reg_write(pdev, speed.channel - 1, CAN_CNF3, &pcnf[2], 1, 0);
  if (ret != 0)
    return ret;
  return 0;
}

static int drv_set_speed_params(TDEV* pdev, unsigned long addr)
{
  int ret;
  SPEED_PARAMS_TTCAN sp;
  uint32_t ch;
  uint8_t cnf1, cnf2, cnf3;

  if (copy_from_user((void*)&sp, (void*)addr, sizeof(sp)) != 0)
    return -EFAULT;
  if (sp.channel != 1 && sp.channel != 2)
    return -EINVAL;
  if (sp.brp < 1 || sp.brp > 32 ||
      sp.sjw < 1 || sp.sjw > 4 ||
      sp.sam > 1 ||
      sp.btlmode > 1 ||
      sp.phseg1 < 1 || sp.phseg1 > 8 ||
      sp.phseg2 < 1 || sp.phseg2 > 8 ||
      sp.prseg < 1 || sp.prseg > 8 ||
      sp.wakfil > 1 ||
      sp.prseg + sp.phseg1 < sp.phseg2 ||
      sp.phseg2 <= sp.sjw)
    return -EINVAL;
  ch = sp.channel - 1;

  cnf1 = ((sp.sjw - 1) << 6) | (sp.brp - 1);
  ret = can_reg_write(pdev, ch, CAN_CNF1, &cnf1, 1, 0);
  if (ret != 0)
    return ret;
  cnf2 = (sp.btlmode << 7) | (sp.sam << 6) | ((sp.phseg1 - 1) << 3) | (sp.prseg - 1);
  ret = can_reg_write(pdev, ch, CAN_CNF2, &cnf2, 1, 0);
  if (ret != 0)
    return ret;
  cnf3 = (1 << 7) | (sp.wakfil << 6) | (sp.phseg2 - 1);
  ret = can_reg_write(pdev, ch, CAN_CNF3, &cnf3, 1, 0);
  if (ret != 0)
    return ret;
  return 0;
}

static int drv_get_errors(TDEV* pdev, unsigned long addr)
{
  int ret;
  ERRORS_TTCAN er;
  uint32_t ch;

  if (copy_from_user((void*)&er, (void*)addr, sizeof(er)) != 0)
    return -EFAULT;
  if (er.channel != 1 && er.channel != 2)
    return -EINVAL;
  ch = er.channel - 1;
  ret = can_reg_read(pdev, ch, EFLG, &er.nEFLG, 1);
  if (ret != 0)
    return ret;
  ret = can_reg_read(pdev, ch, TEC, &er.nTEC, 1);
  if (ret != 0)
    return ret;
  ret = can_reg_read(pdev, ch, REC, &er.nREC, 1);
  if (ret != 0)
    return ret;
  if (copy_to_user((void*)addr, (void*)&er, sizeof(er)) != 0)
    return -EFAULT;
  return 0;
}

static void prepare_data_for_send_buffer(uint8_t pbuf[14], uint8_t prio, uint32_t sid, uint32_t eid, uint8_t* pdata, uint32_t nsize)
{
  pbuf[0] = prio & 0x03;
  pbuf[1] = (uint8_t)(sid >> 3);
  pbuf[2] = (uint8_t)((sid & 0x07) << 5);

  if (eid != (uint32_t)-1)
  {
    pbuf[2] |= (uint8_t)((1 << 3) | ((eid >> 16) & 0x03));
    pbuf[3] = (uint8_t)(eid >> 8);
    pbuf[4] = (uint8_t)eid;
  }
  else
    pbuf[3] = pbuf[4] = 0;

  if (nsize == 0)
    pbuf[5] = 1 << 6;
  else
  {
    pbuf[5] = (uint8_t)nsize;
    memcpy(pbuf + 6, pdata, nsize);
  }
}

static int write_data_to_send_buffer(TDEV* pdev, uint32_t ch, uint8_t nbuf, uint8_t prio, uint32_t sid, uint32_t eid,
                                     uint8_t* pdata, uint32_t nsize, int autorts)
{
  int ret = 0;
  uint8_t buf[14];
  TTXBUF* ptxbuf = &pdev->txbuf[ch][nbuf];
  uint32_t wroffset = 0;
  uint32_t i;

  prepare_data_for_send_buffer(buf, prio, sid, eid, pdata, nsize);

  if (ptxbuf->wrote != 0)
  {
    if (prio == ptxbuf->prio)
    {
      wroffset ++;
      if (sid == ptxbuf->sid)
      {
        wroffset ++;
        if (eid == ptxbuf->eid)
        {
          wroffset += 3;
          if (nsize == ptxbuf->nsize)
          {
            wroffset ++;
            for (i = 0; i < nsize; i ++)
            {
              if (pdata[i] != ptxbuf->data[i])
                break;
              wroffset ++;
            }
          }
        }
      }
    }
  }

  if (wroffset < 6 + nsize)
  {
    ret = can_reg_write(pdev, ch, can_reg_tx_buf[nbuf] + wroffset, buf + wroffset, 6 + nsize - wroffset, autorts);

    if (ret == 0)
    {
      ptxbuf->wrote = 1;
      ptxbuf->prio = prio;
      ptxbuf->sid = sid;
      ptxbuf->eid = eid;
      memcpy(ptxbuf->data, pdata, nsize);
      ptxbuf->nsize = nsize;
    }
    else
      ptxbuf->wrote = 0;
  }
  else if (autorts)
    ret = can_rts(pdev, ch, nbuf);

  return ret;
}

static int drv_write_data_to_tr_buf(TDEV* pdev, unsigned long addr)
{
  int ret;
  SEND_DATA sd;
  uint32_t ch;

  if (copy_from_user((void*)&sd, (void*)addr, sizeof(sd)) != 0)
    return -EFAULT;
  if ((sd.nChannel != 1 && sd.nChannel != 2) ||
      sd.nBufNumber > 2 ||
      sd.nPriority > 3 ||
      sd.SID > 0x7FF ||
      (sd.EID != (uint32_t)-1 && sd.EID > 0x3FFFF) ||
      sd.nSize > 8)
    return -EINVAL;
  ch = sd.nChannel - 1;
  ret = DRV_LOCK_CAN_BUF(pdev, ch, sd.nBufNumber);
  if (ret != 0)
    return ret;
  ret = write_data_to_send_buffer(pdev, ch, sd.nBufNumber, sd.nPriority, sd.SID, sd.EID, sd.nData, sd.nSize, 0);
  DRV_UNLOCK_CAN_BUF(pdev, ch, sd.nBufNumber);
  return ret;
}

static int drv_send_data(TDEV* pdev, unsigned long addr)
{
  int ret;
  SEND_DATA sd;
  uint32_t ch;

  if (copy_from_user((void*)&sd, (void*)addr, sizeof(sd)) != 0)
    return -EFAULT;
  if ((sd.nChannel != 1 && sd.nChannel != 2) ||
      sd.nBufNumber > 2 ||
      sd.nPriority > 3 ||
      sd.SID > 0x7FF ||
      (sd.EID != (uint32_t)-1 && sd.EID > 0x3FFFF) ||
      sd.nSize > 8)
    return -EINVAL;
  ch = sd.nChannel - 1;
  if (pdev->debug_send)
    drv_printf(pdev, "send_data%u.%u start lock\n", ch, sd.nBufNumber);
  ret = DRV_LOCK_CAN_BUF(pdev, ch, sd.nBufNumber);
  if (ret != 0)
    return ret;
  if (pdev->debug_send)
    drv_printf(pdev, "send_data%u.%u got lock\n", ch, sd.nBufNumber);
  reinit_completion(&pdev->xmtd_completion[ch][sd.nBufNumber]);
  ret = write_data_to_send_buffer(pdev, ch, sd.nBufNumber, sd.nPriority, sd.SID, sd.EID, sd.nData, sd.nSize, pdev->disable_autorts ? 0 : 1);
  if (pdev->debug_send)
    drv_printf(pdev, "send_data%u.%u write_data_to_send_buffer done\n", ch, sd.nBufNumber);
  if (ret != 0)
    goto toret;
  if (pdev->disable_autorts)
  {
    ret = can_reg_modify(pdev, ch, can_reg_tx_buf[sd.nBufNumber], 0x0808);
    if (pdev->debug_send)
      drv_printf(pdev, "send_data%u.%u can_reg_modify done\n", ch, sd.nBufNumber);
    if (ret != 0)
      goto toret;
  }

  if (pdev->debug_send)
    drv_printf(pdev, "send_data%u.%u start wait_for_completion_interruptible_timeout\n", ch, sd.nBufNumber);
  ret = wait_for_completion_interruptible_timeout(&pdev->xmtd_completion[ch][sd.nBufNumber], msecs_to_jiffies(sd.timeout));
  if (ret < 0)
  {
    if (pdev->debug_send || pdev->debug_wait)
      drv_printf_err(pdev, "send_data%u.%u wait_for_completion_interruptible_timeout error %d\n", ch, sd.nBufNumber, ret);
    goto toret;
  }
  if (ret == 0)
  {
    if (pdev->debug_send || pdev->debug_wait)
      drv_printf_err(pdev, "send_data%u.%u wait_for_completion_interruptible_timeout timeout\n", ch, sd.nBufNumber);
    ret = can_reg_read(pdev, ch, can_reg_tx_buf[sd.nBufNumber], &sd.txb_ctrl.byte, 1);
    if (ret != 0)
      goto toret;
    if (pdev->debug_halt_on_timeout)
      pdev->halted = 1;
    ret = -ETIME;
    goto toret;
  }
  ret = 0;
toret:
  DRV_UNLOCK_CAN_BUF(pdev, ch, sd.nBufNumber);
  if (pdev->debug_send)
    drv_printf(pdev, "send_data%u.%u leave lock\n", ch, sd.nBufNumber);
  if (ret == -ETIME)
  {
    if (copy_to_user((void*)(addr + ((uint8_t*)&sd.txb_ctrl - (uint8_t*)&sd)), (void*)&sd.txb_ctrl, sizeof(sd.txb_ctrl)) != 0)
      return -EFAULT;
  }
  return ret;
}

static int drv_send_data_now(TDEV* pdev, unsigned long addr)
{
  int ret;
  SEND_DATA_NOW sdn;
  uint32_t ch;

  if (copy_from_user((void*)&sdn, (void*)addr, sizeof(sdn)) != 0)
    return -EFAULT;
  if ((sdn.nChannel != 1 && sdn.nChannel != 2) ||
      sdn.nBuf > 2)
    return -EINVAL;
  ch = sdn.nChannel - 1;
  if (pdev->debug_send)
    drv_printf(pdev, "send_data_now%u.%u start lock\n", ch, sdn.nBuf);
  ret = DRV_LOCK_CAN_BUF(pdev, ch, sdn.nBuf);
  if (ret != 0)
    return ret;
  if (pdev->debug_send)
    drv_printf(pdev, "send_data_now%u.%u got lock\n", ch, sdn.nBuf);
  reinit_completion(&pdev->xmtd_completion[ch][sdn.nBuf]);
  if (pdev->disable_autorts)
  {
    ret = can_reg_modify(pdev, ch, can_reg_tx_buf[sdn.nBuf], 0x0808);
    if (pdev->debug_send)
      drv_printf(pdev, "send_data_now%u.%u can_reg_modify done\n", ch, sdn.nBuf);
  }
  else
  {
    ret = can_rts(pdev, ch, sdn.nBuf);
    if (pdev->debug_send)
      drv_printf(pdev, "send_data_now%u.%u can_rts done\n", ch, sdn.nBuf);
  }
  DRV_UNLOCK_CAN_BUF(pdev, ch, sdn.nBuf);
  if (pdev->debug_send)
    drv_printf(pdev, "send_data_now%u.%u leave lock\n", ch, sdn.nBuf);
  return ret;
}

static int send_data_tg(TDEV* pdev, uint32_t ch, uint8_t nbuf, int epoch, uint32_t epoch_val, uint32_t trig_val, int repeat_mode)
{
  DRV_LOCK_VARS();

  DRV_LOCK(pdev);
  reinit_completion(&pdev->xmtd_completion[ch][nbuf]);
  OUTL(pdev, reg_trig[ch][nbuf], trig_val);
  if (epoch != 0)
    OUTL(pdev, reg_trig_epoch[ch][nbuf], epoch_val);
  INL_AND_OR_OUTL(pdev, reg_trig_ctrl[ch], ~((1 << (13 + nbuf)) | (1 << (9 + nbuf)) | (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0)),
                  (epoch ? (1 << (13 + nbuf)) : 0) | (repeat_mode ? (1 << (9 + nbuf)) : 0) | (1 << (2 + nbuf * 2)));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_send_data_tg(TDEV* pdev, unsigned long addr)
{
  SEND_DATA_TG sdt;

  if (copy_from_user((void*)&sdt, (void*)addr, sizeof(sdt)) != 0)
    return -EFAULT;
  if ((sdt.nChannel != 1 && sdt.nChannel != 2) ||
      sdt.nBuf > 2)
    return -EINVAL;
  return send_data_tg(pdev, sdt.nChannel - 1, sdt.nBuf, sdt.bEpoch, sdt.nEpoch, sdt.nTrigger, 0);
}

static int drv_check_tg(TDEV* pdev, unsigned long addr)
{
  CH_BUF_TTCAN cb;

  if (copy_from_user((void*)&cb, (void*)addr, sizeof(cb)) != 0)
    return -EFAULT;
  if ((cb.nCh != 1 && cb.nCh != 2) ||
      cb.nBuf > 2)
    return -EINVAL;
  return ((INL(pdev, reg_trig_ctrl[cb.nCh - 1]) & (3 << (2 + cb.nBuf * 2))) != 0) ? -EBUSY : 0;
}

static int drv_reset_tg(TDEV* pdev, unsigned long addr)
{
  CH_BUF_TTCAN cb;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&cb, (void*)addr, sizeof(cb)) != 0)
    return -EFAULT;
  if ((cb.nCh != 1 && cb.nCh != 2) ||
      cb.nBuf > 2)
    return -EINVAL;
  ch = cb.nCh - 1;
  DRV_LOCK(pdev);
  INL_AND_OR_OUTL(pdev, reg_trig_ctrl[ch], ~((1 << (13 + cb.nBuf)) | (1 << (9 + cb.nBuf)) | (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0)),
                                           2 << (2 + cb.nBuf * 2));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_send_data_loop(TDEV* pdev, unsigned long addr)
{
  SEND_DATA_TG sdt;

  if (copy_from_user((void*)&sdt, (void*)addr, sizeof(sdt)) != 0)
    return -EFAULT;
  if ((sdt.nChannel != 1 && sdt.nChannel != 2) ||
      sdt.nBuf > 2)
    return -EINVAL;
  return send_data_tg(pdev, sdt.nChannel - 1, sdt.nBuf, sdt.bEpoch, sdt.nEpoch, sdt.nTrigger, 1);
}

static int drv_check_transmit(TDEV* pdev, unsigned long addr)
{
  int ret;
  SEND_DATA_NOW sdn;

  if (copy_from_user((void*)&sdn, (void*)addr, sizeof(sdn)) != 0)
    return -EFAULT;
  if ((sdn.nChannel != 1 && sdn.nChannel != 2) ||
      sdn.nBuf > 2)
    return -EINVAL;
  ret = can_reg_read(pdev, sdn.nChannel - 1, can_reg_tx_buf[sdn.nBuf], &sdn.txb_ctrl.byte, 1);
  if (ret != 0)
    return ret;
  if (copy_to_user((void*)(addr + ((uint8_t*)&sdn.txb_ctrl - (uint8_t*)&sdn)), (void*)&sdn.txb_ctrl, sizeof(sdn.txb_ctrl)) != 0)
    return -EFAULT;
  return (sdn.txb_ctrl.bits.TXREQ != 0) ? -EBUSY : 0;
}

static int drv_wait_transmit(TDEV* pdev, unsigned long addr)
{
  int ret;
  SEND_DATA_NOW sdn;
  uint32_t ch;

  if (copy_from_user((void*)&sdn, (void*)addr, sizeof(sdn)) != 0)
    return -EFAULT;
  if ((sdn.nChannel != 1 && sdn.nChannel != 2) ||
      sdn.nBuf > 2)
    return -EINVAL;
  ch = sdn.nChannel - 1;
  if (pdev->debug_send)
    drv_printf(pdev, "wait_transmit%u.%u start lock\n", ch, sdn.nBuf);
  ret = DRV_LOCK_CAN_BUF(pdev, ch, sdn.nBuf);
  if (ret != 0)
    return ret;
  if (pdev->debug_send)
    drv_printf(pdev, "wait_transmit%u.%u got lock\n", ch, sdn.nBuf);

  if (pdev->debug_send)
    drv_printf(pdev, "wait_transmit%u.%u start wait_for_completion_interruptible_timeout\n", ch, sdn.nBuf);
  ret = wait_for_completion_interruptible_timeout(&pdev->xmtd_completion[ch][sdn.nBuf], msecs_to_jiffies(sdn.timeout));
  if (ret < 0)
  {
    if (pdev->debug_send || pdev->debug_wait)
      drv_printf_err(pdev, "wait_transmit%u.%u wait_for_completion_interruptible_timeout error %d\n", ch, sdn.nBuf, ret);
    goto toret;
  }
  if (ret == 0)
  {
    if (pdev->debug_send || pdev->debug_wait)
      drv_printf_err(pdev, "wait_transmit%u.%u wait_for_completion_interruptible_timeout timeout\n", ch, sdn.nBuf);
    ret = can_reg_read(pdev, ch, can_reg_tx_buf[sdn.nBuf], &sdn.txb_ctrl.byte, 1);
    if (ret != 0)
      goto toret;
    if (pdev->debug_halt_on_timeout)
      pdev->halted = 1;
    ret = -ETIME;
    goto toret;
  }
  ret = 0;
toret:
  DRV_UNLOCK_CAN_BUF(pdev, ch, sdn.nBuf);
  if (pdev->debug_send)
    drv_printf(pdev, "wait_transmit%u.%u leave lock\n", ch, sdn.nBuf);
  if (ret == -ETIME)
  {
    if (copy_to_user((void*)(addr + ((uint8_t*)&sdn.txb_ctrl - (uint8_t*)&sdn)), (void*)&sdn.txb_ctrl, sizeof(sdn.txb_ctrl)) != 0)
      return -EFAULT;
  }
  return ret;
}

static int drv_end_transmit(TDEV* pdev, unsigned long addr)
{
  SEND_DATA_NOW sdn;

  if (copy_from_user((void*)&sdn, (void*)addr, sizeof(sdn)) != 0)
    return -EFAULT;
  if ((sdn.nChannel != 1 && sdn.nChannel != 2) ||
      sdn.nBuf > 2)
    return -EINVAL;
  return can_reg_modify(pdev, sdn.nChannel - 1, can_reg_tx_buf[sdn.nBuf], 0x0008);
}

static int drv_abat(TDEV* pdev, unsigned long addr)
{
  CONF_TTCAN conf;

  if (copy_from_user((void*)&conf, (void*)addr, sizeof(conf)) != 0)
    return -EFAULT;
  if (conf.channel != 1 && conf.channel != 2)
    return -EINVAL;
  return can_reg_modify(pdev, conf.channel - 1, CAN_CTRL, (((conf.mode != 0) ? 0x10 : 0x00) << 8) | 0x10);
}

static int drv_set_timer_trsh(TDEV* pdev, unsigned long addr)
{
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&tt, (void*)addr, sizeof(tt)) != 0)
    return -EFAULT;
  if ((tt.nCh != 1 && tt.nCh != 2) ||
      tt.nEpochBits > 8)
    return -EINVAL;
  ch = tt.nCh - 1;
  DRV_LOCK(pdev);
  OUTL(pdev, reg_timer_trsh[ch], tt.nValue);
  INL_AND_OR_OUTL(pdev, reg_timer_ctrl[ch], ~0xFF4F, (epoch_mask[tt.nEpochBits] << 8) | 0x0B);
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_set_timer_ceed(TDEV* pdev, unsigned long addr)
{
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&tt, (void*)addr, sizeof(tt)) != 0)
    return -EFAULT;
  if (tt.nCh != 1 && tt.nCh != 2)
    return -EINVAL;
  ch = tt.nCh - 1;
  DRV_LOCK(pdev);
  OUTL(pdev, reg_timer_ceed[ch], tt.nValue);
  if (tt.bEpoch != 0)
    OUTL(pdev, reg_epoch_ceed[ch], tt.nEpoch);
  INL_OR_OUTL(pdev, reg_timer_ctrl[ch], (tt.bEpoch ? (1 << 6) : 0) | (1 << 2));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_set_timer_free(TDEV* pdev, unsigned long addr)
{
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&tt, (void*)addr, sizeof(tt)) != 0)
    return -EFAULT;
  if ((tt.nCh != 1 && tt.nCh != 2) ||
      tt.nEpochBits > 8)
    return -EINVAL;
  ch = tt.nCh - 1;
  DRV_LOCK(pdev);
  INL_AND_OR_OUTL(pdev, reg_timer_ctrl[ch],
                  ~((0xFF << 8) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2)),
                  (epoch_mask[tt.nEpochBits] << 8) | (1 << 1) | (1 << 0));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_set_timer_rst_on_rxb(TDEV* pdev, unsigned long addr)
{
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&tt, (void*)addr, sizeof(tt)) != 0)
    return -EFAULT;
  if (tt.nCh != 1 && tt.nCh != 2)
    return -EINVAL;
  ch = tt.nCh - 1;
  DRV_LOCK(pdev);
  INL_AND_OR_OUTL(pdev, reg_timer_ctrl[ch], ~((1 << 5) | (1 << 4)), ((tt.nValue & 1) << 5) | (((tt.nValue >> 1) & 1) << 4));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_stop_timer(TDEV* pdev, unsigned long addr)
{
  uint32_t channel;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&channel, (void*)addr, sizeof(channel)) != 0)
    return -EFAULT;
  if (channel != 1 && channel != 2)
    return -EINVAL;
  ch = channel - 1;
  DRV_LOCK(pdev);
  INL_AND_OUTL(pdev, reg_timer_ctrl[ch], ~(1 << 1));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_get_timer(TDEV* pdev, unsigned long addr)
{
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&tt.nCh, (void*)(addr + ((uint8_t*)&tt.nCh - (uint8_t*)&tt)), sizeof(tt.nCh)) != 0)
    return -EFAULT;
  if (tt.nCh != 1 && tt.nCh != 2)
    return -EINVAL;
  ch = tt.nCh - 1;
  DRV_LOCK(pdev);
  tt.nValue = INL(pdev, reg_timer[ch]);
  tt.nEpoch = INL(pdev, reg_timer_epoch[ch]);
  DRV_UNLOCK(pdev);
  if (copy_to_user((void*)(addr + ((uint8_t*)&tt.nEpoch - (uint8_t*)&tt)), (void*)&tt.nEpoch, sizeof(tt.nEpoch)) != 0)
    return -EFAULT;
  if (copy_to_user((void*)(addr + ((uint8_t*)&tt.nValue - (uint8_t*)&tt)), (void*)&tt.nValue, sizeof(tt.nValue)) != 0)
    return -EFAULT;
  return 0;
}

static int drv_start_timer_int(TDEV* pdev, unsigned long addr)
{
  static const int repeat_mode = 1;
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&tt, (void*)addr, sizeof(tt)) != 0)
    return -EFAULT;
  if (tt.nCh != 1 && tt.nCh != 2)
    return -EINVAL;
  ch = tt.nCh - 1;
  DRV_LOCK(pdev);
  OUTL(pdev, reg_int_trig[ch], tt.nValue);
  if (tt.bEpoch != 0)
    OUTL(pdev, reg_int_trig_epoch[ch], tt.nEpoch);
  INL_AND_OR_OUTL(pdev, reg_trig_ctrl[ch], ~((1 << 12) | (1 << 8) | (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0)),
                  (tt.bEpoch ? (1 << 12) : 0) | (repeat_mode ? (1 << 8) : 0) | (1 << 0));
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_stop_timer_int(TDEV* pdev, unsigned long addr)
{
  uint32_t channel;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&channel, (void*)addr, sizeof(channel)) != 0)
    return -EFAULT;
  if (channel != 1 && channel != 2)
    return -EINVAL;
  ch = channel - 1;
  DRV_LOCK(pdev);
  INL_AND_OR_OUTL(pdev, reg_trig_ctrl[ch], ~((1 << 12) | (1 << 8) | (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0)), 2 << 0);
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_wait_timer_int(TDEV* pdev, unsigned long addr)
{
  int ret;
  TIMER_TRSH_TTCAN tt;
  uint32_t ch;

  if (copy_from_user((void*)&tt, (void*)addr, sizeof(tt)) != 0)
    return -EFAULT;
  if (tt.nCh != 1 && tt.nCh != 2)
    return -EINVAL;
  ch = tt.nCh - 1;

  if (pdev->debug_wait)
    drv_printf(pdev, "wait_timer_int%u start wait_for_completion_interruptible_timeout\n", ch);
  reinit_completion(&pdev->timer_completion[ch]);
  ret = wait_for_completion_interruptible_timeout(&pdev->timer_completion[ch], msecs_to_jiffies(tt.nValue));
  if (ret < 0)
  {
    if (pdev->debug_wait)
      drv_printf_err(pdev, "wait_timer_int%u wait_for_completion_interruptible_timeout error %d\n", ch, ret);
    return ret;
  }
  if (ret == 0)
  {
    if (pdev->debug_wait)
      drv_printf_err(pdev, "wait_timer_int%u wait_for_completion_interruptible_timeout timeout\n", ch);
    if (pdev->debug_halt_on_timeout)
      pdev->halted = 1;
    return -ETIME;
  }
  if (pdev->debug_wait)
    drv_printf(pdev, "wait_timer_int%u ok\n", ch);
  return 0;
}

static int drv_set_timeouts(TDEV* pdev, unsigned long addr)
{
  TIMEOUTS_TTCAN t;
  uint32_t ch;
  DRV_LOCK_VARS();

  if (copy_from_user((void*)&t, (void*)addr, sizeof(t)) != 0)
    return -EFAULT;
  if (t.channel != 1 && t.channel != 2)
    return -EINVAL;
  ch = t.channel - 1;
  DRV_LOCK(pdev);
  OUTL(pdev, reg_timeout_absolute[ch], t.absolute);
  OUTL(pdev, reg_timeout_interval[ch], t.interval);
  DRV_UNLOCK(pdev);
  return 0;
}

static int drv_set_masks(TDEV* pdev, unsigned long addr)
{
  int ret;
  MASKS_TTCAN mask;
  uint32_t ch, nbuf;
  uint32_t sidf, eidf, sidm, eidm;
  uint8_t buff[4];
  uint8_t bufm[4];

  if (copy_from_user((void*)&mask, (void*)addr, sizeof(mask)) != 0)
    return -EFAULT;

  if ((mask.channel != 1 && mask.channel != 2) ||
      mask.rxb_mode >= RXB_MODE_NUM ||
      mask.ident > 1 ||
      mask.filter >= nofels(can_reg_filt))
    return -EINVAL;
  ch = mask.channel - 1;
  nbuf = (mask.filter >= 2) ? 1 : 0;

  sidm = mask.identific_m & 0x7FF;
  eidm = (mask.identific_m >> 11) & 0x3FFFF;
  sidf = mask.identific_f & 0x7FF;
  eidf = (mask.identific_f >> 11) & 0x3FFFF;

  buff[0] = (uint8_t)(sidf >> 3);
  buff[1] = (uint8_t)((sidf & 0x07) << 5);
  buff[2] = (uint8_t)(eidf >> 8);
  buff[3] = (uint8_t)eidf;
  if (mask.ident != 0)
    buff[1] |= (uint8_t)((1 << 3) | ((eidf >> 16) & 0x03));

  bufm[0] = (uint8_t)(sidm >> 3);
  bufm[1] = (uint8_t)((sidm & 0x07) << 5);
  bufm[2] = (uint8_t)(eidm >> 8);
  bufm[3] = (uint8_t)eidm;
  if (mask.ident != 0)
    bufm[1] |= (uint8_t)((eidm >> 16) & 0x03);

  if (mask.rxb_mode != RXB_MODE_OFF)
  {
    ret = can_reg_write(pdev, ch, can_reg_filt[mask.filter], buff, 4, 0);
    if (ret != 0)
      goto toret;
    ret = can_reg_write(pdev, ch, can_reg_mask[nbuf], bufm, 4, 0);
    if (ret != 0)
      goto toret;
  }
  ret = can_reg_modify(pdev, ch, can_reg_rx_buf[nbuf], (mask.rxb_mode << (8 + 5)) | ((RXB_MODE_NUM - 1) << 5));
toret:
  return ret;
}

static int drv_reset_channel(TDEV* pdev, unsigned long addr)
{
  uint32_t nCh;

  if (copy_from_user((void*)&nCh, (void*)addr, sizeof(nCh)) != 0)
    return -EFAULT;
  if (nCh != 1 && nCh != 2)
    return -EINVAL;
  return can_reset(pdev, nCh - 1);
}

static int drv_vers(TDEV* pdev, unsigned long addr)
{
  VERSION_TTCAN ver;

  ver.device_id = pdev->deviceid;
  ver.vendor_id = pdev->vendorid;
  ver.revision = pdev->revisionid;
  memset(ver.dev_name, 0, sizeof(ver.dev_name));
  STRNCPY(ver.dev_name, pdev->devname);
  ver.minor = pdev->unit;
  ver.irq = pdev->pdevice->irq;
  ver.size_dma = TTCAN_DMA_SIZE;
  ver.addr_dma_virt = (uint64_t)(uintptr_t)pdev->dma_vaddr;
  ver.pci_bars = (uint64_t)(uintptr_t)pdev->pbar;
  if (copy_to_user((void*)addr, (void*)&ver, sizeof(ver)) != 0)
    return -EFAULT;
  return 0;
}

static int drv_vers_driver(TDEV* pdev, unsigned long addr)
{
  uint32_t driver_ver;

  driver_ver = DRV_VERSION_AND_DATE;
  if (pdev->debug)
    drv_printf(pdev, "Ver = %X.%X, Date = %02X.%02X.%02X\n",
               driver_ver >> 28, (driver_ver >> 24) & 0x0F,
               (driver_ver >> 16) & 0xFF, (driver_ver >> 8) & 0xFF, driver_ver & 0xFF);
  if (copy_to_user((void*)addr, (void*)&driver_ver, sizeof(driver_ver)))
    return -EFAULT;
  return 0;
}

static int drv_test_wr_reg(TDEV* pdev, unsigned long addr)
{
  SADDR_DATA_MAIN_TTCAN saddr;
  uint32_t i;

  if (copy_from_user((void*)&saddr.daddr, (void*)(addr + ((uint8_t*)&saddr.daddr - (uint8_t*)&saddr)), sizeof(saddr.daddr)) != 0)
    return -EFAULT;
  if (copy_from_user((void*)&saddr.data, (void*)(addr + ((uint8_t*)&saddr.data - (uint8_t*)&saddr)), sizeof(saddr.data)) != 0)
    return -EFAULT;
  for (i = 0; i < 10000000; i ++)
    OUTL(pdev, saddr.daddr, saddr.data);
  return 0;
}

static int drv_test_rd_reg(TDEV* pdev, unsigned long addr)
{
  SADDR_DATA_MAIN_TTCAN saddr;
  uint32_t i;

  if (copy_from_user((void*)&saddr.daddr, (void*)(addr + ((uint8_t*)&saddr.daddr - (uint8_t*)&saddr)), sizeof(saddr.daddr)) != 0)
    return -EFAULT;
  for (i = 0; i < 10000000; i ++)
    saddr.data = INL(pdev, saddr.daddr);
  if (copy_to_user((void*)(addr + ((uint8_t*)&saddr.data - (uint8_t*)&saddr)), (void*)&saddr.data, sizeof(saddr.data)) != 0)
    return -EFAULT;
  return 0;
}

/// \brief адрес укзателя записи дма для канала
#define DMA_WR_ADR(indexChannel)            pdev->dma_wr[indexChannel]
/// \brief адрес укзателя чтения дма для канала
#define DMA_RD_ADR(indexChannel)            pdev->dma_rd[indexChannel]
/// \brief базовый адрес указателя буфера ДМА
#define DMA_VADR                             pdev->dma_vaddr
/// \brief значение базового адреса указателя буфера ДМА
#define VALADR_DMA_VADR                     (uint64_t)DMA_VADR
/// \brief значение адреса конца буфера дма для канала
#define VALADR_DMA_CH_END(indexChannel)     (VALADR_DMA_VADR + indexChannel * SIZE_DMA_CHANNEL_BUFFER + SIZE_DMA_CHANNEL_BUFFER)
/// \brief значение адреса начала буфера дма для канала
#define VALADR_DMA_CH_BEGIN(indexChannel)   (VALADR_DMA_VADR + indexChannel * SIZE_DMA_CHANNEL_BUFFER)
/// \brief значение адреса укзателя записи дма для канала
#define VALADR_DMA_WR(indexChannel)         (uint64_t)DMA_WR_ADR(indexChannel)
/// \brief значение адреса укзателя чтения дма для канала
#define VALADR_DMA_RD(indexChannel)         (uint64_t)DMA_RD_ADR(indexChannel)

static int drv_read_dma_blocks(TDEV* pdev, unsigned long arg)
{
    DMA_READ_BLOCK* block;               // контейнер считываемых блоков dma
    uint32_t dmaIndex = 0;               // индекс dma канала
    int indexChannel = -1;               // индекс канала нормированный
    int readyReadBlocksCount = -1;       // кол-во блоков, готовых для чтения
    int nblockend = -1;                  // кол-во блоков, до правой границы кольцевого буфера
    uint64_t prevRd, prevWrt;
    mutex_lock(&pdev->mtxR);
    copy_from_user((void *)&indexChannel, (void *) arg, sizeof(indexChannel));
    if ((indexChannel != 1) && (indexChannel != 2)) { // проверяем на валидность индекс канала
        printk("NMTTCAN: %s bad numCh = %d", pdev->devname, indexChannel);
        mutex_unlock(&pdev->mtxR);
        return -EFAULT;
    }
    // нормируем индекс канала
    indexChannel = indexChannel - 1;
    // чтение индекса dma канала
    switch (indexChannel) {
    case 0:
        dmaIndex = ioread32(pdev->pbar + OFFSET_DMA1_INDEX_REG_TTCAN);
        break;
    case 1:
        dmaIndex = ioread32(pdev->pbar + OFFSET_DMA2_INDEX_REG_TTCAN);
        break;
    }
    if (dmaIndex == 0xFFFFFFFF) {
        printk("NMCAN: %s bad dma_index_reg = 0x%x", pdev->devname, dmaIndex);
        mutex_unlock(&pdev->mtxR);
        return -EFAULT;
    }
    dmaIndex = dmaIndex >> 6;
    prevRd = (uint64_t)DMA_RD_ADR(indexChannel);
    prevWrt = (uint64_t)DMA_WR_ADR(indexChannel);
    // обновляем указатель записи
    DMA_WR_ADR(indexChannel)  = (uint32_t*)(DMA_VADR + indexChannel * SIZE_DMA_CHANNEL_BUFFER + dmaIndex * DMA_RAW_BLOCK_SIZE_64);
    // считаем кол-во готовых к чтению блоков
    if ( VALADR_DMA_WR(indexChannel) >= VALADR_DMA_RD(indexChannel) ) { // не превышает границу буфера
        readyReadBlocksCount = (VALADR_DMA_WR(indexChannel) - VALADR_DMA_RD(indexChannel)) / DMA_RAW_BLOCK_SIZE_64;
    } else {
        readyReadBlocksCount = (VALADR_DMA_WR(indexChannel) - VALADR_DMA_CH_BEGIN(indexChannel)) / DMA_RAW_BLOCK_SIZE_64 + \
                (VALADR_DMA_CH_END(indexChannel) - VALADR_DMA_RD(indexChannel)) / DMA_RAW_BLOCK_SIZE_64;
    }
    if (readyReadBlocksCount == 0) {
        copy_to_user((void*)(arg + sizeof(int)), (void*)&readyReadBlocksCount, sizeof(int));
        mutex_unlock(&pdev->mtxR);
        return 0;
    }
    block = kzalloc(sizeof(DMA_READ_BLOCK), GFP_KERNEL);
    copy_from_user((void *)block, (void *) arg, sizeof(DMA_READ_BLOCK)); // получаем данные из юзерспейс
    block->info.prevRd = prevRd;
    block->info.prevWrt = prevWrt;
    block->info.dmaIndex = dmaIndex;
    if (readyReadBlocksCount >= DMA_COUNT_BLOCKS)
        block->countBlocks = DMA_COUNT_BLOCKS;
    else
        block->countBlocks = readyReadBlocksCount;
    // вычисляем кол-во блоков, до границы буфера
    nblockend = (VALADR_DMA_CH_END(indexChannel) - VALADR_DMA_RD(indexChannel)) / DMA_RAW_BLOCK_SIZE_64; // вычисляем кол-во блоков, до границы буфера
//    printk("NMTTCAN: %s numCh = %d, dmaIndex = %d, readyReadBlocksCount = %d, countBlocks = %d, nblockend = %d",
//           pdev->devname, block->numChannel, dmaIndex, readyReadBlocksCount, block->countBlocks, nblockend);
    switch (indexChannel) {
    case 0:
        dmaIndex = ioread32(pdev->pbar + OFFSET_DMA1_RD_INDEX_REG_TTCAN);
        dmaIndex = dmaIndex >> 6;
        break;
    case 1:
        dmaIndex = ioread32(pdev->pbar + OFFSET_DMA2_RD_INDEX_REG_TTCAN);
        dmaIndex = dmaIndex >> 6;
        break;
    }
    dmaIndex += block->countBlocks;
    if (dmaIndex >= NUMBER_MSG_IN_DMA_CHANNEL_BUFFER)
        dmaIndex = dmaIndex - NUMBER_MSG_IN_DMA_CHANNEL_BUFFER;
//    printk("NMTTCAN: %s dmaIndex = %d", pdev->devname, dmaIndex);
    switch (indexChannel) {
    case 0:
        iowrite32(dmaIndex<<6, pdev->pbar + OFFSET_DMA1_RD_INDEX_REG_TTCAN);
        break;
    case 1:
        iowrite32(dmaIndex<<6, pdev->pbar + OFFSET_DMA2_RD_INDEX_REG_TTCAN);
        break;
    }
    // считываем блоки
    if (block->countBlocks <= nblockend) { // кол-во запрошенных блоков не превышает границу буфера
        memcpy((void*)block->blocks, (void*)DMA_RD_ADR(indexChannel), block->countBlocks * DMA_RAW_BLOCK_SIZE_64); // копируем в юзерспейс блоки
        DMA_RD_ADR(indexChannel) += block->countBlocks * DMA_RAW_BLOCK_SIZE_64 / 4; // сдвигаем указатель чтения
        if (VALADR_DMA_RD(indexChannel) > VALADR_DMA_CH_END(indexChannel))
            DMA_RD_ADR(indexChannel) = (uint32_t*)(DMA_VADR + indexChannel * SIZE_DMA_CHANNEL_BUFFER); // при необходимости корректируем
//        printk("NMTTCAN: %s NOT OVER BORDER", pdev->devname);
    } else {
        memcpy((void*)block->blocks, (void*)DMA_RD_ADR(indexChannel), nblockend * DMA_RAW_BLOCK_SIZE_64);  // копируем первый кусок блоков
        DMA_RD_ADR(indexChannel) = (uint32_t*)(DMA_VADR + indexChannel * SIZE_DMA_CHANNEL_BUFFER);
        memcpy((void*)( (uint8_t*)block->blocks + nblockend * DMA_RAW_BLOCK_SIZE_64), (void*)DMA_RD_ADR(indexChannel), (block->countBlocks - nblockend) * DMA_RAW_BLOCK_SIZE_64); // копируем второй кусок блоков
        DMA_RD_ADR(indexChannel) = (uint32_t*)(DMA_VADR + indexChannel * SIZE_DMA_CHANNEL_BUFFER + (block->countBlocks - nblockend) * DMA_RAW_BLOCK_SIZE_64);
//        printk("NMTTCAN: %s OVER BORDER", pdev->devname);
    }   
//    printk("\r");
    block->info.rd = (uint64_t)DMA_RD_ADR(indexChannel);
    block->info.wrt = (uint64_t)DMA_WR_ADR(indexChannel);
    copy_to_user((void*)arg, (void*)block, sizeof(DMA_READ_BLOCK));  // копируем в юзерспейc
    kfree(block);
    mutex_unlock(&pdev->mtxR);
    return 0;
}

static int drv_write_data_to_fifo1(TDEV* pdev, unsigned long arg)
{
    CAN_WRT_MSG* msg;           // сообщение для записи
    uint64_t adrBuf = 0;        // адрес регистра
    uint32_t word = 0;          // слово данных
    mutex_lock(&pdev->mtxT1);
    msg = kzalloc(sizeof(CAN_WRT_MSG), GFP_KERNEL);
    copy_from_user((void *)msg, (void *)arg, sizeof(CAN_WRT_MSG)); // получаем данные из юзерспейс
    adrBuf = (msg->mTypeFifo == CAN_WRT_MSG_TYPE_FIFO) ? REG_TTCAN1_BUF : REG_TTCAN1_HP_FIFO;
    // запись сообщения в буфер словами по 32 бита
    word = 0;
    word = (((msg->mSID >> 3) & 0xFF) << (0*8))
            | ((((msg->mSID << 5) | ((msg->mEID >> 16) & 0x3) | (((msg->mIsExtMsg == CAN_WRT_MSG_EXT_MSG_NO) ? 0:1) << 3)) & 0xFF) << (1*8))
            | (((msg->mEID >> 8) & 0xFF) << (2*8))
            | (((msg->mEID) & 0xFF) << (3*8))
            ; // sidh | sidl | eid8 | eid0
    iowrite32(word, pdev->pbar + adrBuf + 0x0);
    word = 0;
    word |= (((msg->mDataLength & 0xF) & 0xFF) << (0*8)); // dls
    if (((msg->mDataLength & 0xF) - 1) >= 0) word |= (uint32_t)(msg->mData[0]) << (1*8); // data0
    if (((msg->mDataLength & 0xF) - 1) >= 1) word |= (uint32_t)(msg->mData[1]) << (2*8); // data1
    if (((msg->mDataLength & 0xF) - 1) >= 2) word |= (uint32_t)(msg->mData[2]) << (3*8); // data2
    iowrite32(word, pdev->pbar + adrBuf + 0x4);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 3) word |= (uint32_t)(msg->mData[3]) << (0*8); // data3
    if (((msg->mDataLength & 0xF) - 1) >= 4) word |= (uint32_t)(msg->mData[4]) << (1*8); // data4
    if (((msg->mDataLength & 0xF) - 1) >= 5) word |= (uint32_t)(msg->mData[5]) << (2*8); // data5
    if (((msg->mDataLength & 0xF) - 1) >= 6) word |= (uint32_t)(msg->mData[6]) << (3*8); // data6
    iowrite32(word, pdev->pbar + adrBuf + 0x8);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 7) word |= (uint32_t)(msg->mData[7]) << (0*8); // data7
    word |= (uint32_t)(msg->mMsgId) << (1*8); // msgid
    iowrite32(word, pdev->pbar + adrBuf + 0xC);
    kfree(msg);
    mutex_unlock(&pdev->mtxT1);
    return 0;
}

static int drv_write_data_to_fifo2(TDEV* pdev, unsigned long arg)
{
    CAN_WRT_MSG* msg;           // сообщение для записи
    uint64_t adrBuf = 0;        // адрес регистра
    uint32_t word = 0;          // слово данных
    mutex_lock(&pdev->mtxT2);
    msg = kzalloc(sizeof(CAN_WRT_MSG), GFP_KERNEL);
    copy_from_user((void *)msg, (void *)arg, sizeof(CAN_WRT_MSG)); // получаем данные из юзерспейс
    adrBuf = (msg->mTypeFifo == CAN_WRT_MSG_TYPE_FIFO) ? REG_TTCAN2_BUF : REG_TTCAN2_HP_FIFO;
    // запись сообщения в буфер словами по 32 бита
    word = 0;
    word = (((msg->mSID >> 3) & 0xFF) << (0*8))
            | ((((msg->mSID << 5) | ((msg->mEID >> 16) & 0x3) | (((msg->mIsExtMsg == CAN_WRT_MSG_EXT_MSG_NO) ? 0:1) << 3)) & 0xFF) << (1*8))
            | (((msg->mEID >> 8) & 0xFF) << (2*8))
            | (((msg->mEID) & 0xFF) << (3*8))
            ; // sidh | sidl | eid8 | eid0
    iowrite32(word, pdev->pbar + adrBuf + 0x0);
    word = 0;
    word |= (((msg->mDataLength & 0xF) & 0xFF) << (0*8)); // dls
    if (((msg->mDataLength & 0xF) - 1) >= 0) word |= (uint32_t)(msg->mData[0]) << (1*8); // data0
    if (((msg->mDataLength & 0xF) - 1) >= 1) word |= (uint32_t)(msg->mData[1]) << (2*8); // data1
    if (((msg->mDataLength & 0xF) - 1) >= 2) word |= (uint32_t)(msg->mData[2]) << (3*8); // data2
    iowrite32(word, pdev->pbar + adrBuf + 0x4);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 3) word |= (uint32_t)(msg->mData[3]) << (0*8); // data3
    if (((msg->mDataLength & 0xF) - 1) >= 4) word |= (uint32_t)(msg->mData[4]) << (1*8); // data4
    if (((msg->mDataLength & 0xF) - 1) >= 5) word |= (uint32_t)(msg->mData[5]) << (2*8); // data5
    if (((msg->mDataLength & 0xF) - 1) >= 6) word |= (uint32_t)(msg->mData[6]) << (3*8); // data6
    iowrite32(word, pdev->pbar + adrBuf + 0x8);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 7) word |= (uint32_t)(msg->mData[7]) << (0*8); // data7
    word |= (uint32_t)(msg->mMsgId) << (1*8); // msgid
    iowrite32(word, pdev->pbar + adrBuf + 0xC);
    kfree(msg);
    mutex_unlock(&pdev->mtxT2);
    return 0;
}

static int drv_write_data_to_fifo1_V2(TDEV* pdev, unsigned long arg)
{
    CAN_WRT_MSG_V2* msg;           // сообщение для записи
    uint64_t adrBuf = 0;        // адрес регистра
    uint32_t word = 0;          // слово данных
    mutex_lock(&pdev->mtxT1);
    msg = kzalloc(sizeof(CAN_WRT_MSG_V2), GFP_KERNEL);
    copy_from_user((void *)msg, (void *)arg, sizeof(CAN_WRT_MSG_V2)); // получаем данные из юзерспейс
    adrBuf = (msg->mTypeFifo == CAN_WRT_MSG_TYPE_FIFO) ? REG_TTCAN1_BUF : REG_TTCAN1_HP_FIFO;
    // запись сообщения в буфер словами по 32 бита
    word = 0;
    word = (((msg->mSID >> 3) & 0xFF) << (0*8))
            | ((((msg->mSID << 5) | ((msg->mEID >> 16) & 0x3) | (((msg->mIsExtMsg == CAN_WRT_MSG_EXT_MSG_NO) ? 0:1) << 3)) & 0xFF) << (1*8))
            | (((msg->mEID >> 8) & 0xFF) << (2*8))
            | (((msg->mEID) & 0xFF) << (3*8))
            ; // sidh | sidl | eid8 | eid0
    iowrite32(word, pdev->pbar + adrBuf + 0x0);
    word = 0;
    word |= (( (msg->mDataLength & 0xF) | ( (msg->mRtrBit) ? 1<<6 : 0 ) ) << (0*8)); // dls n RTR
    if (((msg->mDataLength & 0xF) - 1) >= 0) word |= (uint32_t)(msg->mData[0]) << (1*8); // data0
    if (((msg->mDataLength & 0xF) - 1) >= 1) word |= (uint32_t)(msg->mData[1]) << (2*8); // data1
    if (((msg->mDataLength & 0xF) - 1) >= 2) word |= (uint32_t)(msg->mData[2]) << (3*8); // data2
    iowrite32(word, pdev->pbar + adrBuf + 0x4);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 3) word |= (uint32_t)(msg->mData[3]) << (0*8); // data3
    if (((msg->mDataLength & 0xF) - 1) >= 4) word |= (uint32_t)(msg->mData[4]) << (1*8); // data4
    if (((msg->mDataLength & 0xF) - 1) >= 5) word |= (uint32_t)(msg->mData[5]) << (2*8); // data5
    if (((msg->mDataLength & 0xF) - 1) >= 6) word |= (uint32_t)(msg->mData[6]) << (3*8); // data6
    iowrite32(word, pdev->pbar + adrBuf + 0x8);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 7) word |= (uint32_t)(msg->mData[7]) << (0*8); // data7
    word |= (uint32_t)(msg->mMsgId) << (1*8); // msgid
    iowrite32(word, pdev->pbar + adrBuf + 0xC);
    kfree(msg);
    mutex_unlock(&pdev->mtxT1);
    return 0;
}

static int drv_write_data_to_fifo2_V2(TDEV* pdev, unsigned long arg)
{
    CAN_WRT_MSG_V2* msg;           // сообщение для записи
    uint64_t adrBuf = 0;        // адрес регистра
    uint32_t word = 0;          // слово данных
    mutex_lock(&pdev->mtxT2);
    msg = kzalloc(sizeof(CAN_WRT_MSG_V2), GFP_KERNEL);
    copy_from_user((void *)msg, (void *)arg, sizeof(CAN_WRT_MSG_V2)); // получаем данные из юзерспейс
    adrBuf = (msg->mTypeFifo == CAN_WRT_MSG_TYPE_FIFO) ? REG_TTCAN2_BUF : REG_TTCAN2_HP_FIFO;
    // запись сообщения в буфер словами по 32 бита
    word = 0;
    word = (((msg->mSID >> 3) & 0xFF) << (0*8))
            | ((((msg->mSID << 5) | ((msg->mEID >> 16) & 0x3) | (((msg->mIsExtMsg == CAN_WRT_MSG_EXT_MSG_NO) ? 0:1) << 3)) & 0xFF) << (1*8))
            | (((msg->mEID >> 8) & 0xFF) << (2*8))
            | (((msg->mEID) & 0xFF) << (3*8))
            ; // sidh | sidl | eid8 | eid0
    iowrite32(word, pdev->pbar + adrBuf + 0x0);
    word = 0;
    word |= (( (msg->mDataLength & 0xF) | ( (msg->mRtrBit) ? 1<<6 : 0 ) ) << (0*8)); // dls n RTR
    if (((msg->mDataLength & 0xF) - 1) >= 0) word |= (uint32_t)(msg->mData[0]) << (1*8); // data0
    if (((msg->mDataLength & 0xF) - 1) >= 1) word |= (uint32_t)(msg->mData[1]) << (2*8); // data1
    if (((msg->mDataLength & 0xF) - 1) >= 2) word |= (uint32_t)(msg->mData[2]) << (3*8); // data2
    iowrite32(word, pdev->pbar + adrBuf + 0x4);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 3) word |= (uint32_t)(msg->mData[3]) << (0*8); // data3
    if (((msg->mDataLength & 0xF) - 1) >= 4) word |= (uint32_t)(msg->mData[4]) << (1*8); // data4
    if (((msg->mDataLength & 0xF) - 1) >= 5) word |= (uint32_t)(msg->mData[5]) << (2*8); // data5
    if (((msg->mDataLength & 0xF) - 1) >= 6) word |= (uint32_t)(msg->mData[6]) << (3*8); // data6
    iowrite32(word, pdev->pbar + adrBuf + 0x8);
    word = 0;
    if (((msg->mDataLength & 0xF) - 1) >= 7) word |= (uint32_t)(msg->mData[7]) << (0*8); // data7
    word |= (uint32_t)(msg->mMsgId) << (1*8); // msgid
    iowrite32(word, pdev->pbar + adrBuf + 0xC);
    kfree(msg);
    mutex_unlock(&pdev->mtxT2);
    return 0;
}

static long drv_ioctl(struct file* pfile, uint32_t cmd, unsigned long addr)
{
  TDEV* pdev;

  pdev = (TDEV*)pfile->private_data;
  if (pdev == NULL)
    return -ENXIO;

  if (pdev->halted)
    return -EIO;

  switch (cmd)
  {
    case IOCTL_WR_MAINREG_TTCAN :
      return drv_wr_reg(pdev, addr);
    case IOCTL_RD_MAINREG_TTCAN :
      return drv_rd_reg(pdev, addr);
    case IOCTL_WR_CANREG_TTCAN :
      return drv_wr_canreg(pdev, addr);
    case IOCTL_RD_CANREG_TTCAN :
      return drv_rd_canreg(pdev, addr);
    case IOCTL_MODIFY_CANREG_TTCAN :
      return drv_modify_canreg(pdev, addr);
    case IOCTL_WR_CANREGS_TTCAN :
      return drv_wr_canregs(pdev, addr);
    case IOCTL_RD_CANREGS_TTCAN :
      return drv_rd_canregs(pdev, addr);
    case IOCTL_ENABLE_DMA_TTCAN :
      return drv_enable_dma(pdev, addr);
    case IOCTL_DISABLE_DMA_TTCAN :
      return drv_disable_dma(pdev, addr);
    case IOCTL_RD_CH_RAW_DMA_TTCAN :
      return drv_rd_ch_raw_dma(pdev, addr); //<<<<<<<<<<<<<<<<<<
    case IOCTL_SET_MODE_TTCAN :
      return drv_set_mode(pdev, addr);
    case IOCTL_GET_MODE_TTCAN :
      return drv_get_mode(pdev, addr);
    case IOCTL_SET_ONESHOT_MODE_TTCAN :
      return drv_set_oneshot_mode(pdev, addr);
    case IOCTL_SET_SPEED_TTCAN :
      return drv_set_speed(pdev, addr);
    case IOCTL_SET_SPEED_PARAMS_TTCAN :
      return drv_set_speed_params(pdev, addr);
    case IOCTL_GET_ERRORS_TTCAN :
      return drv_get_errors(pdev, addr);
    case IOCTL_WRITE_DATA_TO_TR_BUF_TTCAN :
      return drv_write_data_to_tr_buf(pdev, addr);
    case IOCTL_SEND_DATA_TTCAN :
      return drv_send_data(pdev, addr);
    case IOCTL_SEND_DATA_NOW_TTCAN :
      return drv_send_data_now(pdev, addr);
    case IOCTL_SEND_DATA_TG_TTCAN :
      return drv_send_data_tg(pdev, addr);
    case IOCTL_SEND_DATA_LOOP_TTCAN :
      return drv_send_data_loop(pdev, addr);
    case IOCTL_CHECK_TG_TTCAN :
      return drv_check_tg(pdev, addr);
    case IOCTL_RESET_TG_TTCAN :
      return drv_reset_tg(pdev, addr);
    case IOCTL_CHECK_TRANSMIT_TTCAN :
      return drv_check_transmit(pdev, addr);
    case IOCTL_WAIT_TRANSMIT_TTCAN :
      return drv_wait_transmit(pdev, addr);
    case IOCTL_END_TRANSMIT_TTCAN :
      return drv_end_transmit(pdev, addr);
    case IOCTL_ABAT_TTCAN :
      return drv_abat(pdev, addr);
    case IOCTL_SET_CANn_TIMER_TRSH_TTCAN :
      return drv_set_timer_trsh(pdev, addr);
    case IOCTL_SET_CANn_TIMER_CEED_TTCAN :
      return drv_set_timer_ceed(pdev, addr);
    case IOCTL_SET_CANn_TIMER_FREE_TTCAN :
      return drv_set_timer_free(pdev, addr);
    case IOCTL_SET_CANn_TIMER_RST_ON_RXB_TTCAN :
      return drv_set_timer_rst_on_rxb(pdev, addr);
    case IOCTL_STOP_CANn_TIMER_TTCAN :
      return drv_stop_timer(pdev, addr);
    case IOCTL_GET_CANn_TIMER_TTCAN :
      return drv_get_timer(pdev, addr);
    case IOCTL_START_CANn_TIMER_INT_TTCAN :
      return drv_start_timer_int(pdev, addr);
    case IOCTL_STOP_CANn_TIMER_INT_TTCAN :
      return drv_stop_timer_int(pdev, addr);
    case IOCTL_WAIT_CANn_TIMER_INT_TTCAN :
      return drv_wait_timer_int(pdev, addr);
    case IOCTL_SET_CANn_TIMEOUTS_TTCAN :
      return drv_set_timeouts(pdev, addr);
    case IOCTL_SET_MASKS_TTCAN :
      return drv_set_masks(pdev, addr);
    case IOCTL_RESET_TTCAN :
      return drv_reset(pdev);
    case IOCTL_RESET_CANn_TTCAN :
      return drv_reset_channel(pdev, addr);
    case IOCTL_VERSION_TTCAN :
      return drv_vers(pdev, addr);
    case IOCTL_VERSION_DRIVER_TTCAN :
      return drv_vers_driver(pdev, addr);
    case IOCTL_TEST_WR_MAINREG_TTCAN :
      return drv_test_wr_reg(pdev, addr);
    case IOCTL_TEST_RD_MAINREG_TTCAN :
      return drv_test_rd_reg(pdev, addr);
  case IOCTL_READ_DMA_BLOCKS_TTCAN:
     return drv_read_dma_blocks(pdev, addr);
  case IOCTL_WRITE_DATA_TO_FIFO_TTCAN1:
      return drv_write_data_to_fifo1(pdev, addr);
  case IOCTL_WRITE_DATA_TO_FIFO_TTCAN2:
      return drv_write_data_to_fifo2(pdev, addr);
  case IOCTL_WRITE_DATA_TO_FIFO_TTCAN1_V2:
      return drv_write_data_to_fifo1_V2(pdev, addr);
  case IOCTL_WRITE_DATA_TO_FIFO_TTCAN2_V2:
      return drv_write_data_to_fifo2_V2(pdev, addr);
    default :
      break;
  }

  return -ENOTTY;
}
