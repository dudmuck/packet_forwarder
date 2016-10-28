## simple LoRaWAN server running on gateway ##

This server only provides answers to end-node for diagnostic purpose.  No UDP networking, no GPS.

The end node configuration must be added into the json configuration file, in the `"motes"` array under `"lorawan"`.  Example is provided in the json files here.
For ABP end-nodes, `dev_addr` is given along with session keys.  For OTA end-nodes, the `dev_eui`, `app_eui` and `app_key` is given.
