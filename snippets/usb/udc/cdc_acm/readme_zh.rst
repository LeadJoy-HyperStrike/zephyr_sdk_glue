.. _cdc_acm:

cdc_acm
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/cdc_acm/README.html>`_

路径
---------------

.. code-block::

    zephyr/samples/subsys/usb/cdc_acm

命令行
-----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S cdc_acm samples/subsys/usb/cdc_acm -T sample.usb_device_next.cdc-acm

As hpm5100evk for example:

.. code-block:: console

    west build -p always -b hpm5100evk -S cdc_acm zephyr/samples/subsys/usb/cdc_acm -T sample.usb_device_next.cdc-acm -d build_hpm5100evk_usb_cdc_acm
    west flash -d build_hpm5100evk_usb_cdc_acm

hpm5100evk 说明：

- 使用板载 USB0 数据口（不是调试器 USB 口）。
- 调试串口一般为 COM @ 115200；枚举成功后 Windows 会出现 CDC 虚拟串口。
- 驱动默认按 CherryUSB 走 **HS + SLE**；snippet 因 32KB HRAM 强制加大栈并 ``CONFIG_DCACHE=n``。
