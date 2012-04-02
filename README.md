In case anyone other than me reads this. There are a lot of things to do. Adding
all neccessary drivers from older kernels and adapting the interfaces. This is
just a list of things I noticed or is not important to fix now.

If you stumple over this and are interested to help then feel free to contact me.

I won't guarantee that I continue to work on this.

### Things to check
 * BUG_ON() seems to hang the device and pulling the battery is neccessary
 * acpuclk_init() does not differentiate between GSM and CDMA devices
 * Is adding smd_set_channel_list() the correct way now?
 * USB stuff looks very different
   * board_serialno() ?
 * Microp?
