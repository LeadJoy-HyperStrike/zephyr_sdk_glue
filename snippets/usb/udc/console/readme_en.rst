.. _console:

console
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/console/README.html>`_

Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/console

Build Cmd
------------

As hpm5100evk for example:

.. code-block:: console

    west build -p always -b hpm5100evk -S console zephyr/samples/subsys/usb/console -d build_hpm5100evk_usb_console -- -DCONF_FILE=usbd_next_prj.conf
    west flash -d build_hpm5100evk_usb_console

Notes:

- Use board USB0; open the virtual COM with DTR enabled to see periodic ``Hello World!``.
- Debug UART is no longer the console (chosen points to CDC ACM).
