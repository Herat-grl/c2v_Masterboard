/***************************************************************************//**
*  \file       driver.c
*
*  \details    Simple Linux device driver (Signals)
*
*
* *******************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>                 //kmalloc()
#include <linux/uaccess.h>              //copy_to/from_user()
#include <linux/ioctl.h>
#include <linux/interrupt.h>
//#include <linux/sched/signal.h> /*<linux/signal.h> was moved to <linux/sched/signal.h> in the 4.11 kernel,*/
#include <asm/io.h>
#include <linux/gpio.h>
/**<linux/signal.h> was moved to <linux/sched/signal.h> in the 4.11 kernel, 
* so you need to conditionally include one or the other depending on the kernel version:*/
#include <linux/version.h>  
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
#include <linux/signal.h>
#else
#include <linux/sched/signal.h>
#endif
 
#define TEGRA194_AON_GPIO_PORT_BB       446                 //ingpio - pd Intr (Jeton Header Pin#31)
#define TEGRA194_MAIN_GPIO_PORT_B       389                 //outgpio - pd Intr clr (Jeton Header Pin#29)

#define SIGETX 44

#define REG_CURRENT_TASK _IOW('a','a',int32_t*)
 
#define IRQ_NO 11
 
/* Signaling to Application */
static struct task_struct *task = NULL;
static int signum = 0;
 
 
dev_t dev = 0;
static struct class *dev_class;
static struct cdev etx_cdev;
struct siginfo info;
 
static int __init etx_driver_init(void);
static void __exit etx_driver_exit(void);
static int etx_open(struct inode *inode, struct file *file);
static int etx_release(struct inode *inode, struct file *file);
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len,loff_t * off);
static ssize_t etx_write(struct file *filp, const char *buf, size_t len, loff_t * off);
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
 
static struct file_operations fops =
{
        .owner          = THIS_MODULE,
        .read           = etx_read,
        .write          = etx_write,
        .open           = etx_open,
        .unlocked_ioctl = etx_ioctl,
        .release        = etx_release,
};

#if 0 
//Interrupt handler for IRQ 11. 
static irqreturn_t irq_handler(int irq,void *dev_id) {
    int err_stat, lgpioval = 0;

    printk(KERN_INFO "Shared IRQ: Interrupt Occurred");
    
    //Sending signal to app

    printk(KERN_INFO "task = %d \n",task);

    lgpioval = gpio_get_value(TEGRA194_AON_GPIO_PORT_BB);
    printk(KERN_INFO "lgpioval = %d \n",lgpioval);
	
    if(!lgpioval) /**Interrupt Active Low*/
    {
    //gpio_direction_output(TEGRA194_MAIN_GPIO_PORT_B,0); /**Once read the IRQ_RC_REG then clear the interrupt handled in the user space*/
    if (task != NULL) {
        printk(KERN_INFO "Sending signal to app\n");

	err_stat = send_sig_info(SIGETX, &info, task);
        if(err_stat < 0)
	{
            printk(KERN_INFO "Unable to send signal Error = %d \n", err_stat);
        }
    //gpio_direction_output(TEGRA194_MAIN_GPIO_PORT_B,1); /**Once read the IRQ_RC_REG then clear the interrupt*/
    }
    }
 	
    return IRQ_HANDLED;
}
#endif
 
static int etx_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "Device File Opened...!!!\n");
    return 0;
}
 
static int etx_release(struct inode *inode, struct file *file)
{
    struct task_struct *ref_task = get_current();
    printk(KERN_INFO "Device File Closed...!!!\n");
    
    //delete the task
    if(ref_task == task) {
        task = NULL;
    }
    return 0;
}

static ssize_t etx_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    printk(KERN_INFO "Write Function\n");
    //asm("int $0x3B");  //Triggering Interrupt. Corresponding to irq 11
    
    return 0;
}

static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    int value, result;
    char data_buf[5];
    printk(KERN_INFO "Write function\n");

    if (copy_from_user(&data_buf, buf, len))  // copy
    {
        result = -EFAULT;
        printk("copy_from_user: Err!\n");
     return -1;
    }
    
    switch (data_buf[1])
    {
        case 1:
            if(data_buf[0]) {
                gpio_request(445, "ingpio");
                gpio_request(256, "outgpio");
                gpio_direction_input(445);
                gpio_direction_output(256, data_buf[0]);
            } else {
                gpio_request(445, "ingpio");
                gpio_request(256, "outgpio");
                gpio_direction_input(445);
                gpio_direction_output(256, data_buf[0]);
            }
            value = gpio_get_value(445);
            gpio_free(445);
            gpio_free(256);
            break;
        case 2:
            if(data_buf[0]) {
                gpio_request(446, "ingpio");
                gpio_request(389, "outgpio");
                gpio_direction_input(446);
                gpio_direction_output(389, data_buf[0]);
            } else {
                gpio_request(446, "ingpio");
                gpio_request(389, "outgpio");
                gpio_direction_input(446);
                gpio_direction_output(389, data_buf[0]);
            }
            value = gpio_get_value(446);
            gpio_free(446);
            gpio_free(389);
            break;
        default:
            printk("invalid pin set\n");
            return -1;
    }
    
    printk("value : %d\n", value);
    if(value == data_buf[0]) {
        printk("pass gpio test...\n");
        result = 1;
    } else {
        printk("fail gpio test...\n");
        result = 0;
    }
    
    if (copy_to_user(buf, &result, 1)) {
        pr_err("Data Read : Err!\n");
      }

    return 0;
}
 
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct pid *pid_struct = NULL;

    if (cmd == REG_CURRENT_TASK) {
	printk(KERN_INFO "arg = %ld \n",arg);
	pid_struct = find_get_pid(arg);
        printk(KERN_INFO "REG_CURRENT_TASK\n");

	task = pid_task(pid_struct,PIDTYPE_PID);
	printk(KERN_INFO "task = %d \n",task);
	if(task == NULL)
	{
	   	printk ("Cannot find pid from user program\r\n");
   		return -1;
	}

    memset(&info, 0, sizeof(struct siginfo));
    info.si_signo = SIGETX;
    info.si_code = SI_QUEUE;
    info.si_int = 1;

        signum = SIGETX;
    }
    return 0;
}
 
 
static int __init etx_driver_init(void)
{
    int irq;
    /*Allocating Major number*/
    if((alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) <0){
            printk(KERN_INFO "Cannot allocate major number\n");
            return -1;
    }
    printk(KERN_INFO "Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
 
    /*Creating cdev structure*/
    cdev_init(&etx_cdev,&fops);
 
    /*Adding character device to the system*/
    if((cdev_add(&etx_cdev,dev,1)) < 0){
        printk(KERN_INFO "Cannot add the device to the system\n");
        goto r_class;
    }
 
    /*Creating struct class*/
    if((dev_class = class_create(THIS_MODULE,"etx_class")) == NULL){
        printk(KERN_INFO "Cannot create the struct class\n");
        goto r_class;
    }
 
    /*Creating device*/
    if((device_create(dev_class,NULL,dev,NULL,"phy_intr")) == NULL){
        printk(KERN_INFO "Cannot create the Device 1\n");
        goto r_device;
    }
 

    // gpio_request(TEGRA194_AON_GPIO_PORT_BB, "ingpio");
    // gpio_request(TEGRA194_MAIN_GPIO_PORT_B, "outgpio");

    // gpio_direction_input(TEGRA194_AON_GPIO_PORT_BB);
 //   gpio_direction_output(TEGRA194_MAIN_GPIO_PORT_B,1);

    // irq=gpio_to_irq(TEGRA194_AON_GPIO_PORT_BB);

    // if (request_irq(irq, irq_handler, IRQF_TRIGGER_FALLING, "phy_intr", &dev)) {
    //     printk(KERN_INFO "my_device: cannot register IRQ ");
    //     goto irq;
    // }
 
    printk(KERN_INFO "Device Driver Insert...Done!!!\n");
    return 0;
// irq:
//     free_irq(irq,(void *)(irq_handler));
r_device:
    class_destroy(dev_class);
r_class:
    unregister_chrdev_region(dev,1);
    return -1;
}
 
static void __exit etx_driver_exit(void)
{
    int irq;
    //irq=gpio_to_irq(TEGRA194_AON_GPIO_PORT_BB);	
    //free_irq(irq,&dev);
    gpio_free(TEGRA194_MAIN_GPIO_PORT_B);
    gpio_free(TEGRA194_AON_GPIO_PORT_BB);
    device_destroy(dev_class,dev);
    class_destroy(dev_class);
    cdev_del(&etx_cdev);
    unregister_chrdev_region(dev, 1);
    printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}
 
module_init(etx_driver_init);
module_exit(etx_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple device driver - Signals");
MODULE_VERSION("1.20");
