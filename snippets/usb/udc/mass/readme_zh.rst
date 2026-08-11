.. _mass:

mass
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/mass/README.html>`_

路径
------

.. code-block::

    zephyr/samples/subsys/usb/mass

命令行
------------

以hpm6750evk2为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S mass samples/subsys/usb/mass -T sample.usb_device_next.mass_ram_none

以hpm5100evk为例（RAM 盘在 DLM，约 64KB）:

.. code-block:: console

    west build -p always -b hpm5100evk -S mass zephyr/samples/subsys/usb/mass -d build_hpm5100evk_usb_mass
    west flash -d build_hpm5100evk_usb_mass

hpm5100evk 说明：

- 系统 RAM 仍为 32KB HRAM；RAM 盘经 ``ram-region`` 放在 **DLM 后 64KB**，避免挤爆 HRAM。
- 此前 4KB 盘 Windows **无法 FAT 格式化**；64KB 后可格式化。掉电内容丢失属正常。
- 使用板载 USB0 数据口。
