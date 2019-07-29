#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/signal.h>
#include <linux/pm.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#define MW_SENSOR_NAME "mwsensor"

#define MW_SENSOR_NEAR 1
#define MW_SENSOR_FAR  2

// ioctl cmd
#define MW_SENSOR_IOC_MAGIC  'm'

#define MW_SENSOR_IOC_ENABLE _IOW(MW_SENSOR_IOC_MAGIC, 1, int)
#define MW_SENSOR_IOC_SET_RATE _IOW(MW_SENSOR_IOC_MAGIC, 2, int)

#define MW_SENSOR_IOC_MAXNR 2


struct mw_sensor_data {
    struct platform_device  *platform_dev;
    struct miscdevice       mw_sensor_device;
    struct input_dev        *input_dev;
    struct notifier_block   notifier;
    
    int                     irq;
    int                     irq_gpio;
    unsigned long           suspend_time;
    bool                    is_suspended;
    u32                     is_delay;
    u32                     is_poll;
    u32                     delay_time;
    bool                    enabled;
};

static inline uint32_t gettime_now(void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return tv.tv_sec;
}

static void mw_sensor_report_event(struct mw_sensor_data *mw_sensor, s32 data)
{
    struct input_dev *input = mw_sensor->input_dev;

    if (!mw_sensor->enabled) {
        return;
    }

    input_report_rel(input, REL_MISC, data);
    input_sync(input);
}

static int mw_sensor_enable(struct mw_sensor_data *mw_sensor) {
    mw_sensor->enabled = true;
    if (gpio_get_value(mw_sensor->irq_gpio)) {
         mw_sensor_report_event(mw_sensor, MW_SENSOR_NEAR); 
    } else {
        mw_sensor_report_event(mw_sensor, MW_SENSOR_FAR); 
    }
    return 0;
}

static int mw_sensor_disable(struct mw_sensor_data *mw_sensor) {
    mw_sensor->enabled = false;
    return 0;
}

static int mw_sensor_suspend(struct mw_sensor_data *mw_sensor)
{
    dev_info(&mw_sensor->platform_dev->dev, "mw_sensor_suspend\n");
    mw_sensor->suspend_time = gettime_now();
    mw_sensor->is_suspended = true;
    //enable_irq_wake(mw_sensor->irq);
    
    return 0;
}

static int mw_sensor_resume(struct mw_sensor_data *mw_sensor)
{
    dev_info(&mw_sensor->platform_dev->dev, "mw_sensor_resume\n");
    //mw_sensor->is_suspended = false;
    //disable_irq_wake(mw_sensor->irq);

    return 0;
}

static int mw_sensor_fb_event_notify(struct notifier_block *noti,
                                     unsigned long event, void *data)
{
    int *blank;
    struct fb_event *ev_data = data;
    struct mw_sensor_data *mw_sensor = container_of(noti,
                                struct mw_sensor_data, notifier);

    if (ev_data && ev_data->data && event == FB_EVENT_BLANK && mw_sensor) {
        blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
            mw_sensor_resume(mw_sensor);
        } else if (*blank == FB_BLANK_POWERDOWN) {
            mw_sensor_suspend(mw_sensor);
        }
    }

    return NOTIFY_OK;
}

static void mw_sensor_wakeup(struct mw_sensor_data *mw_sensor) {
    dev_info(&mw_sensor->platform_dev->dev, "%s, wake up\n", __func__);
    
    input_report_key(mw_sensor->input_dev, KEY_POWER, 1);
    input_sync(mw_sensor->input_dev);
    input_report_key(mw_sensor->input_dev, KEY_POWER, 0);
    input_sync(mw_sensor->input_dev);
}

static irqreturn_t mw_sensor_irq_handle(int irq, void *dev_id)
{
    struct mw_sensor_data *mw_sensor = dev_id;
    unsigned long cur_time = gettime_now();

    // dts中配置是否延时，因为待机过程会产生干扰，导致微波Sensor误触发
    if (!mw_sensor->is_delay || abs(cur_time - mw_sensor->suspend_time) > mw_sensor->delay_time) {
        if (gpio_get_value(mw_sensor->irq_gpio)) {
            if (mw_sensor->is_suspended) {
                mw_sensor->is_suspended = false;
                mw_sensor_wakeup(mw_sensor);
                msleep(100);
                mw_sensor_report_event(mw_sensor, MW_SENSOR_NEAR);
            } else {
                mw_sensor_report_event(mw_sensor, MW_SENSOR_NEAR); 
            }
        } else {
            mw_sensor_report_event(mw_sensor, MW_SENSOR_FAR); 
        }
    }

    return IRQ_HANDLED;
}

static void mw_sensor_free_irq(struct mw_sensor_data *mw_sensor)
{
    if(mw_sensor) {
        free_irq(mw_sensor->irq, mw_sensor);
    }
}

static void mw_sensor_free_io_port(struct mw_sensor_data *mw_sensor)
{
    if(gpio_is_valid(mw_sensor->irq_gpio)) {
        gpio_free(mw_sensor->irq_gpio);
    }
    return;
}

static int mw_sensor_parse_dt(struct device *dev,
                              struct mw_sensor_data *mw_sensor)
{
    int ret = 0;
    struct device_node *np = dev->of_node;

    mw_sensor->irq_gpio = of_get_named_gpio(np, "irq-gpios", 0);
    if(!gpio_is_valid(mw_sensor->irq_gpio)) {
        dev_err(dev, "No valid irq gpio");
        return -1;
    }

    ret = of_property_read_u32(np, "mwsensor,is_delay", &mw_sensor->is_delay);
    if (ret) {
        mw_sensor->is_delay = 0;
    }

    ret = of_property_read_u32(np, "mwsensor,delay_time", &mw_sensor->delay_time);
    if (ret) {
        mw_sensor->delay_time = 3;
    }

    ret = of_property_read_u32(np, "mwsensor,is_poll", &mw_sensor->is_poll);
    if (ret) {
        mw_sensor->is_poll = 0;
    }

    dev_info(dev, "mw_sensor_parse_dt, is_delay=%d is_poll=%d delay_time=%d\n", 
        mw_sensor->is_delay, mw_sensor->is_poll, mw_sensor->delay_time);

    return 0;
}

static int mw_sensor_request_io_port(struct mw_sensor_data *mw_sensor)
{
    int ret = 0;

    if(gpio_is_valid(mw_sensor->irq_gpio)) {
        ret = gpio_request(mw_sensor->irq_gpio, "mw_sensor_int");

        if(ret < 0) {
            dev_err(&mw_sensor->platform_dev->dev,
                    "Failed to request GPIO:%d, ERRNO:%d\n",
                    (s32)mw_sensor->irq_gpio, ret);
            return -ENODEV;
        }

        gpio_direction_input(mw_sensor->irq_gpio);
        dev_info(&mw_sensor->platform_dev->dev, "Success request irq-gpio\n");
    }

    return ret;
}

static s8 mw_sensor_request_input_dev(struct mw_sensor_data *mw_sensor)
{
    s8 ret = -1;

    mw_sensor->input_dev = input_allocate_device();
    if(mw_sensor->input_dev == NULL) {
        dev_err(&mw_sensor->platform_dev->dev, "Failed to allocate input device\n");
        return -ENOMEM;
    }

    mw_sensor->input_dev->name = MW_SENSOR_NAME;
    input_set_drvdata(mw_sensor->input_dev, mw_sensor);

    input_set_capability(mw_sensor->input_dev, EV_KEY, KEY_POWER);
    input_set_capability(mw_sensor->input_dev, EV_REL, REL_MISC);

    ret = input_register_device(mw_sensor->input_dev);
    if(ret) {
        dev_err(&mw_sensor->platform_dev->dev, "Register %s input device failed\n",
                mw_sensor->input_dev->name);
        input_free_device(mw_sensor->input_dev);
        return -ENODEV;
    }

    return 0;
}

static int mw_sensor_dev_open(struct inode *inode, struct file *filp)
{
    int ret = 0;

    struct mw_sensor_data *mw_sensor = container_of(filp->private_data,
                                  struct mw_sensor_data,
                                  mw_sensor_device);
    filp->private_data = mw_sensor;
    dev_info(&mw_sensor->platform_dev->dev,
             "device node major=%d, minor=%d\n", imajor(inode), iminor(inode));

    return ret;
}

static long mw_sensor_dev_ioctl(struct file *pfile,
                                 unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int data = 0;
    struct mw_sensor_data *mw_sensor = pfile->private_data;

    if (_IOC_TYPE(cmd) != MW_SENSOR_IOC_MAGIC) {
        return -EINVAL;
    }
    if (_IOC_NR(cmd) > MW_SENSOR_IOC_MAXNR) {
        return -EINVAL;
    }

    if (_IOC_DIR(cmd) & _IOC_READ) {
        ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
    } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
        ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
    }
    if (ret) {
        return -EFAULT;
    }

    if (copy_from_user(&data, (int *)arg, sizeof(int))) {
        dev_err(&mw_sensor->platform_dev->dev,
                "%s, copy from user failed\n", __func__);
        return -EFAULT;
    }

    dev_info(&mw_sensor->platform_dev->dev,
             "%s, (%x, %lx): data=%d\n", __func__, cmd,
             arg, data);

    switch (cmd) {
        case MW_SENSOR_IOC_ENABLE:
            if (data > 0) {
                mw_sensor_enable(mw_sensor);
            } else {
                mw_sensor_disable(mw_sensor);
            }
            break;

        case MW_SENSOR_IOC_SET_RATE:
            break;

        default:
            return -EINVAL;
    }

    return ret;
}

static const struct file_operations mw_sensor_dev_fops = {
    .owner = THIS_MODULE,
    .open = mw_sensor_dev_open,
    .unlocked_ioctl = mw_sensor_dev_ioctl
};

static int mw_sensor_request_irq(struct mw_sensor_data *mw_sensor)
{
    int ret = 0;

    /* use irq */
    if(gpio_is_valid(mw_sensor->irq_gpio) || mw_sensor->irq > 0) {
        if(gpio_is_valid(mw_sensor->irq_gpio))
            mw_sensor->irq = gpio_to_irq(mw_sensor->irq_gpio);

        dev_info(&mw_sensor->platform_dev->dev, "INT num %d, trigger type:%d\n",
                 mw_sensor->irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
        ret = request_threaded_irq(mw_sensor->irq, NULL,
                                   mw_sensor_irq_handle,
                                   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                   mw_sensor->platform_dev->name,
                                   mw_sensor);

        if(ret < 0) {
            dev_err(&mw_sensor->platform_dev->dev,
                    "Failed to request irq %d\n", mw_sensor->irq);
        }
    }

    return ret;
}

static int mw_sensor_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct mw_sensor_data *mw_sensor;

    mw_sensor = devm_kzalloc(&pdev->dev, sizeof(*mw_sensor), GFP_KERNEL);
    if(mw_sensor == NULL) {
        dev_err(&pdev->dev, "Failed alloc ts memory\n");
        return -ENOMEM;
    }

    if(pdev->dev.of_node) {
        ret = mw_sensor_parse_dt(&pdev->dev, mw_sensor);
        if(ret) {
            dev_err(&pdev->dev, "Failed parse dts\n");
            goto exit_free_data;
        }
    }

    mw_sensor->platform_dev = pdev;

    ret = mw_sensor_request_io_port(mw_sensor);
    if(ret < 0) {
        dev_err(&pdev->dev, "Failed request IO port\n");
        goto exit_free_data;
    }

    ret = mw_sensor_request_input_dev(mw_sensor);
    if(ret < 0) {
        dev_err(&pdev->dev, "Failed request IO port\n");
        goto exit_free_io_port;
    }

    ret = mw_sensor_request_irq(mw_sensor);
    if(ret < 0) {
        dev_err(&pdev->dev, "Failed create work thread\n");
        goto exit_unreg_input_dev;
    }

    ret = enable_irq_wake(mw_sensor->irq);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed set irq wake\n");
        goto exit_free_irq;
    }

    mw_sensor->mw_sensor_device.minor = MISC_DYNAMIC_MINOR;
    mw_sensor->mw_sensor_device.name = MW_SENSOR_NAME;
    mw_sensor->mw_sensor_device.fops = &mw_sensor_dev_fops;
    ret = misc_register(&mw_sensor->mw_sensor_device);
    if (ret) {
        dev_err(&pdev->dev, "Failed to misc_register\n");
        goto exit_free_irq;
    }

    mw_sensor->notifier.notifier_call = mw_sensor_fb_event_notify;
    fb_register_client(&mw_sensor->notifier);

    platform_set_drvdata(pdev, mw_sensor);

    dev_info(&pdev->dev, "%s, over\n", __func__);
    return 0;

exit_free_irq:
    free_irq(mw_sensor->irq, mw_sensor);
    
exit_unreg_input_dev:
    input_unregister_device(mw_sensor->input_dev);
    
exit_free_io_port:
    if(gpio_is_valid(mw_sensor->irq_gpio))
        gpio_free(mw_sensor->irq_gpio);
    
exit_free_data:
    devm_kfree(&pdev->dev, mw_sensor);

    return ret;
}

static int mw_sensor_remove(struct platform_device *pdev)
{
    struct mw_sensor_data *mw_sensor = platform_get_drvdata(pdev);
    input_unregister_device(mw_sensor->input_dev);
    input_free_device(mw_sensor->input_dev);
    fb_unregister_client(&mw_sensor->notifier);
    mw_sensor_free_irq(mw_sensor);
    mw_sensor_free_io_port(mw_sensor);
    kfree(mw_sensor);

    return 0;
}

static const struct of_device_id mw_sensor_of_match[] = {
    { .compatible =  "topband,mwsensor"},
    {},
};

MODULE_DEVICE_TABLE(of, mw_sensor_of_match);

static struct platform_driver mw_sensor_driver = {
    .probe = mw_sensor_probe,
    .remove = mw_sensor_remove,
    .driver = {
        .name = MW_SENSOR_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = mw_sensor_of_match,
    },
};

module_platform_driver(mw_sensor_driver);

MODULE_AUTHOR("shenhb@topband.com.cn");
MODULE_DESCRIPTION("Microware Sensor");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
