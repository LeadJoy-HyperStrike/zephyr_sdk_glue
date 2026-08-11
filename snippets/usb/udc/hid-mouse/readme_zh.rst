.. _hid-mouse:

hid-mouse
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/hid-mouse/README.html>`_

路径
------

.. code-block::

    zephyr/samples/subsys/usb/hid-mouse

命令行
------------

以hpm6750evk2为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-mouse samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse

以hpm5100evk为例:

.. code-block:: console

    west build -p always -b hpm5100evk -S hid-mouse zephyr/samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse -d build_hpm5100evk_usb_hid_mouse
    west flash -d build_hpm5100evk_usb_hid_mouse

hpm5100evk 说明：

- 使用板载 USB0 数据口；按键为 ``sw0``（左键），LED 为 ``led0``。
- overlay 对 ``gpio_keys`` 开启 ``polling-mode``（HPM GPIO 不支持 ``GPIO_INT_EDGE_BOTH``）。

已知问题
----------

- 由于hpm6750/hpm6800 gpio不支持 `GPIO_INT_EDGE_BOTH` 中断,因此不支持gpio按键功能