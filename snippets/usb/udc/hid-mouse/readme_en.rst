.. _hid-mouse:

hid-mouse
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/hid-mouse/README.html>`_

Sample Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/hid-mouse

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-mouse samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse

As hpm5100evk for example:

.. code-block:: console

    west build -p always -b hpm5100evk -S hid-mouse zephyr/samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse -d build_hpm5100evk_usb_hid_mouse
    west flash -d build_hpm5100evk_usb_hid_mouse

Notes for hpm5100evk:

- Use board USB0 data port; ``sw0`` is left click, ``led0`` is the sample LED.
- Overlay enables ``polling-mode`` on ``gpio_keys`` (HPM GPIO lacks ``GPIO_INT_EDGE_BOTH``).

Known Issues
-------------

- As hpm6750/hpm6800 gpio don't support `GPIO_INT_EDGE_BOTH` interrupt, so gpio-keys function is not supported.