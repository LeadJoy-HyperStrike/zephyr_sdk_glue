/* HPM51 HS2: preemptible thread-only controller transfers. No per-byte IRQ,
 * no global interrupt mask, no stale ADDRHIT, bounded by wall time rather
 * than CPU loop counts (USB deadline preemption must not cause timeouts).
 * Touch protocols deliberately use separate STOP-terminated messages. */
#if defined(CONFIG_HS2_HPM51_TOUCH_BOOT_DIAG)
extern void h51_tp_boot_i2c(uint16_t addr, uint8_t flags, uint32_t len, int rc,
	uint32_t status, uint32_t ctrl, uint32_t cmd, uint32_t remain, const uint8_t *data);
#define H51_I2C_TRACE(rc) h51_tp_boot_i2c(addr,m->flags,m->len,rc,p->STATUS,p->CTRL,p->CMD,i2c_get_data_count(p),m->buf)
#else
#define H51_I2C_TRACE(rc) do { } while (0)
#endif
static int hpm51_master_poll(const struct device *dev,struct i2c_msg *m,uint16_t addr)
{
	const struct hpmicro_i2c_config *cfg=dev->config;
	I2C_Type *p=cfg->base;
	if(!(m->flags&I2C_MSG_STOP)||(m->flags&I2C_MSG_ADDR_10_BITS)||addr>127||m->len>I2C_SOC_TRANSFER_COUNT_MAX)
		return -ENOTSUP;
	bool read=(m->flags&I2C_MSG_RW_MASK)==I2C_MSG_READ;
	uint32_t budget=k_ms_to_cyc_ceil32(20+((m->len+1)*180u+999u)/1000u);
	uint32_t start=k_cycle_get_32();
	int rc=-ETIMEDOUT;
	while(p->STATUS&I2C_STATUS_BUSBUSY_MASK)if((uint32_t)(k_cycle_get_32()-start)>budget)goto abort;
	i2c_disable_irq(p,I2C_EVENT_ALL_MASK);
	i2c_enable_10bit_address_mode(p,false);
	i2c_clear_status(p,I2C_EVENT_ALL_MASK);
	p->CMD=I2C_CMD_CLEAR_FIFO;p->ADDR=I2C_ADDR_ADDR_SET(addr);
	uint32_t ctrl=I2C_CTRL_PHASE_START_MASK|I2C_CTRL_PHASE_ADDR_MASK|I2C_CTRL_PHASE_STOP_MASK|
		I2C_CTRL_DIR_SET(read?I2C_DIR_MASTER_READ:I2C_DIR_MASTER_WRITE);
	if(m->len) {
		ctrl|=I2C_CTRL_PHASE_DATA_MASK|I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(m->len));
#ifdef I2C_CTRL_DATACNT_HIGH_MASK
		ctrl|=I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(m->len)>>8);
#endif
	}
	p->CTRL=ctrl;p->CMD=I2C_CMD_ISSUE_DATA_TRANSMISSION;
	bool hit=false;uint32_t n=0;
	for(;;) {
		uint32_t st=p->STATUS;
		if(st&I2C_STATUS_ADDRHIT_MASK)hit=true;
		if(hit && n<m->len) {
			if(read && !(st&I2C_STATUS_FIFOEMPTY_MASK))m->buf[n++]=(uint8_t)p->DATA;
			else if(!read && !(st&I2C_STATUS_FIFOFULL_MASK))p->DATA=m->buf[n++];
		}
		if(st&I2C_STATUS_CMPL_MASK) {
			if(!hit){rc=-ENXIO;goto abort;}
			/* A completed read can still have its last FIFO bytes pending. */
			if(read && n<m->len && !(p->STATUS&I2C_STATUS_FIFOEMPTY_MASK))continue;
			if(n==m->len && i2c_get_data_count(p)==0) {
				H51_I2C_TRACE(0);
				i2c_clear_status(p,I2C_EVENT_ALL_MASK);return 0;
			}
			rc=-EIO;goto abort;
		}
		if((uint32_t)(k_cycle_get_32()-start)>budget)goto abort;
	}
abort:
	H51_I2C_TRACE(rc);
	/* STOP then reset the controller on failure, without touching other buses. */
	i2c_clear_status(p,I2C_EVENT_ALL_MASK);
	p->CTRL=I2C_CTRL_PHASE_STOP_MASK;p->CMD=I2C_CMD_ISSUE_DATA_TRANSMISSION;
	start=k_cycle_get_32();
	while((p->CMD&I2C_CMD_CMD_MASK)==I2C_CMD_ISSUE_DATA_TRANSMISSION &&
	      (uint32_t)(k_cycle_get_32()-start)<k_us_to_cyc_ceil32(200)) {}
	i2c_config_t bus={.i2c_mode=i2c_mode_fast,.is_10bit_addressing=false};
	(void)i2c_init_master(p,clock_get_frequency(cfg->clock_name),&bus);
	return rc;
}
