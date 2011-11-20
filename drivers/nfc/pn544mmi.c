 /* support temp nfc test before the HAL and the Up-level code
 * is added to our project.
 * It is re-written according to a test software
 * provided by the NXP company who is the manufacturer of pn544
 * Maybe we will remove this file later.
 */ 


#include <linux/nfc/pn544.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/seq_file.h>

/* we add pn544_debug_control to let the 
 * driver can control the printk more easier.
 */
#define PN544_MMI_DEBUG(message, ...) \
	do { \
	if (pn544_debug_mask && pn544_debug_control) \
		printk(message, ## __VA_ARGS__); \
	} while (0)

static int pn544_mmi_proc_show(struct seq_file *m, void *v)
{
	int ret=0;
	char pn544_mmi_return[50];

	ret=pn544_hci_exec(pn544_mmi_return);
	seq_printf(m,"%s\n",pn544_mmi_return);
	return 0;
}

static int pn544_mmi_open(struct inode *inode,struct file *file)
{
	return single_open(file, pn544_mmi_proc_show, NULL);
}


static const struct file_operations pn544_mmi_fops = 
{
	.open 			= pn544_mmi_open,
	.release		= single_release,
	.llseek			= seq_lseek,
	.read 			= seq_read,
};

int pn544_mmi_init(void)
{
	struct proc_dir_entry *pn544_dir;
	struct proc_dir_entry *pn544_mmi;
	PN544_MMI_DEBUG("%s:entered and ready to create proc node to support MMI test\n",__func__);

	pn544_dir=proc_mkdir("pn544",NULL);
	if(NULL==pn544_dir)
	{
		printk("Can not create folder pn544 in proc system.\n");
		return -EIO;
	}
	pn544_mmi=proc_create("pn544_mmi", S_IRUGO, pn544_dir, &pn544_mmi_fops);
	if(NULL==pn544_mmi)
	{
		printk("Can not create folder pn544 in proc system.\n");
		return -EIO;
	}	
	return 0;
}

