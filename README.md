# esp32_rmt_sw_serial
this is a proof of concept whether we can use RMT as Software Serial; A: it depends

it works for short messages even with 8 channels (tested with 7)
the point is: RMT capture edges until the RMT buffer is full or idle is detected, than it blocks the receiver in the ISR and
copies the data to the ringbuffer. So your messages needs to be shorter than the RMT buffer.
You have 512 x 32bits and you can split that to 8 channels.
Worst case of your message content is 0xAA as it has the most edges.
With 8N1 would have 10 edges resulting in 5 entries. To be on the save side
your messages should be shorter as 100 bytes devided by the number of channels you want to use.
And be aware of adapting the idle timer.

this codes includes a lot of debugging, you might remove it. and I had trouble with splitting up the 
code in .h an .cpp - don't know why yet.
