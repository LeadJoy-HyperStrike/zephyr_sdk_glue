.. _hid_keyboard:

hid_keyboard
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/hid-keyboard/README.html>`_

Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/hid-keyboard

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-keyboard samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard

As hpm5100evk for example:

.. code-block:: console

    west build -p always -b hpm5100evk -S hid-keyboard zephyr/samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard -d build_hpm5100evk_usb_hid_keyboard
    west flash -d build_hpm5100evk_usb_hid_keyboard

Notes for hpm5100evk:

- Use board USB0 data port; ``sw0`` / ``led0`` are used by the sample.
- Overlay enables ``polling-mode`` on ``gpio_keys`` (HPM GPIO lacks ``GPIO_INT_EDGE_BOTH``).

Known Issues
-------------

- As hpm6750/hpm6800 gpio don't support `GPIO_INT_EDGE_BOTH` interrupt, so gpio-keys function is not supported.