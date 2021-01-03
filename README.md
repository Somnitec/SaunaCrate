SaunaCrate

An open source DIY sauna build.

Current features:
-Build inside of a plywood shipping container suitable for international cargo. Size ca 1x1.1x1.8m ~= 2m3 volume (was free with another project we did)
-Vapor barier by flooring foam with a metal foil layer. (ca €20)
-inside wood with cheap spruce wood (ca €150)
-3kw 230V sauna heater (ca €120)
-RGB neonflex lighting with a slight red/orange color variation
-Arduino nano controller, with 2x10a relay and si7021 sensor
-off the shelf mechanical thermometer, hygrometer and sand timer

Software:
-Heating with hysteresis
-2h shutdown timer, resettable with a button
-overheat switch-off

Features potentially planned
-overheat fuse (should shut down the relays at 130oC even if arduino tries to keep them on)
-nicer enclosures for the electronics
-wifi connected for temperature monitoring and remote start (esp32?)
-more light modes (potentially have a timer be reflected in the lights)
-color therapy theory implemented in the led colors
-sound system integrated
-lots of mechanical/physical improvements