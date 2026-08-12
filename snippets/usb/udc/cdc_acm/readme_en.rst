.. _cdc_acm:

cdc_acm
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/cdc_acm/README.html>`_

Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/cdc_acm

Build Cmd
-----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S cdc_acm samples/subsys/usb/cdc_acm -T sample.usb_device_next.cdc-acm

As hpm5100evk for example:

.. code-block:: console

    west build -p always -b hpm5100evk -S cdc_acm zephyr/samples/subsys/usb/cdc_acm -T sample.usb_device_next.cdc-acm -d build_hpm5100evk_usb_cdc_acm
    west flash -d build_hpm5100evk_usb_cdc_acm

Notes for hpm5100evk:

- Use board USB0 data port (not the debugger USB port).
- Debug UART is typically COM @ 115200; CDC appears as a Windows virtual COM after enumeration.
- Driver defaults to **HS + SLE** (CherryUSB-aligned); snippet uses large stacks and ``CONFIG_DCACHE=n`` for 32KB HRAM.
