.. _console:

console
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/console/README.html>`_

路径
------

.. code-block::

    zephyr/samples/subsys/usb/console

命令行
------------

以hpm5100evk为例:

.. code-block:: console

    west build -p always -b hpm5100evk -S console zephyr/samples/subsys/usb/console -d build_hpm5100evk_usb_console -- -DCONF_FILE=usbd_next_prj.conf
    west flash -d build_hpm5100evk_usb_console

说明：

- 使用板载 USB0；Windows 出现虚拟 COM 后打开串口工具（开 DTR），应周期性打印 ``Hello World!``。
- 调试 UART 不再作 console（chosen 指向 CDC ACM）。
