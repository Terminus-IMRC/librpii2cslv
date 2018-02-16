# `librpii2cslv`

`librpii2cslv` is a I2C client library for Raspberry Pi.


## Requirements

You need [wiringPi](http://wiringpi.com/download-and-install/) to use this
library.


## Installation

```
$ autoreconf -i -m
$ ./configure
$ make
$ sudo make install
```


## Testing

```
$ make check
$ sudo test/test_simple
```
