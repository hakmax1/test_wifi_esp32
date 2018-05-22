ESP-IDF template app
====================

This is a template application to be used with `Espressif IoT Development Framework`_ (ESP-IDF). 

Please check ESP-IDF docs for getting started instructions.

Code in this repository is Copyright (C) 2016 Espressif Systems, licensed under the Apache License 2.0 as described in the file LICENSE.

.. _Espressif IoT Development Framework: https://github.com/espressif/esp-idf


http GET httpbin.org fff

http GET http://httpbin.org/get fff

HTTP/1.1 200 OK
Connection: close
Server: gunicorn/19.8.1
Date: Mon, 21 May 2018 20:02:03 GMT
Content-Type: application/json
Content-Length: 157
Access-Control-Allow-Origin: *
Access-Control-Allow-Credentials: true
Via: 1.1 vegur

{"args":{},"headers":{"Connection":"close","Host":"httpbin.org","User-Agent":"esp-idf/1.0 esp32"},"origin":"178.150.178.107","url":"http://httpbin.org/get"}
