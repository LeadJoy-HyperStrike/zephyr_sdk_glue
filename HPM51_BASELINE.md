# HPM51 SDK1.13 集成基线

长期分支 `leadjoy/hs2-hpm5100` 从远端 `leadjoy/hs2-hpm6e00` 的 `d725b739c1dea0b4bc99ca090369729f9593a354` 派生，保留其HPM5100新供应商投放、SDK1.13适配、I2C消息计数修复、PMA/cache防护与其它MCU的条件分支。

移入旧51产品提交eb80e3b中的USB单核快速通路、HRAM DMA缓冲/端点取消、可抢占I2C轮询、SPI错误收尾、非阻塞UART poll和受限CPU idle配置。6E双核fastpath源码保留，两套端点所有权配置不能同时开启。正式51配置关闭额外统计和UDC latency hook。

HS2的CONFIG_HS2_HPM51_CLOCK_400保留已验证800MHz PLL0 /2 CPU /2 AHB配方，得到400/200MHz，不进行电压或OTP写入；其它板默认仍是上游720MHz /1.8配方。默认QTD池缩小只限HPM5100，不影响6E。

**HS2 51必须使用固件仓库的west-hpm51.yml**，锁定本分支提交与官方sdk_env v1.13.0。这里west.yml保留集成线上游入口，不作为HS2 51应用的依赖入口；不会再从相邻ER SDK目录借文件。SDK1.13组合的实板时延/升级/断电持久性仍待用户另行授权验证。

原6E集成分支不修改。源码和许可保留，没有密钥/OTP数据。当前仅本地准备，尚未推送。
