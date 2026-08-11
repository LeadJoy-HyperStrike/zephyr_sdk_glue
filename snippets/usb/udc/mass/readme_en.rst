.. _mass:

mass
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/mass/README.html>`_

Sample Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/mass

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S mass samples/subsys/usb/mass -T sample.usb_device_next.mass_ram_none

As hpm5100evk for example (64KB RAM disk in DLM):

.. code-block:: console

    west build -p always -b hpm5100evk -S mass zephyr/samples/subsys/usb/mass -d build_hpm5100evk_usb_mass
    west flash -d build_hpm5100evk_usb_mass

Notes for hpm5100evk:

- System RAM stays on 32KB HRAM; the RAM disk is placed in the **upper 64KB of DLM** via ``ram-region``.
- A 4KB disk is too small for Windows FAT format; 64KB can be formatted. Contents are lost on power-off.
- Use board USB0 data port.
