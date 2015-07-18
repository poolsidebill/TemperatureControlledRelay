Temperature Controlled AC Relay

This project was created so I could automatically control an attic fan in my garage when the Texas temperature
got too high. The fan just plugs into this project's controlled AC outlet.

I wanted to use a 8pin ATtiny85 AVR to perform the logic since it had a WatchDog timer, low current, and enough I/O pins
to do the job. Temperature readings were performed by reading a low-cost LM34 sensor. I used a old cell phone charger
to provide power, and a 5v electronic relay to control 120vac to an AC outlet my fan was plugged into.

Added:<br>
pushbutton - triggers AC power on for 30mins<br>
LED - flashes every 4sec to let me know everything is still working<br>
AC Outlet Box - everything fits in this double-wide box<br>

I programmed the ATtiny85 via Ardunio IDE software calls. I used an Ardunio UNO as my ISP to upload teh compiled code to
the Flash memory in the ATtiny85. See this link for a good rundown on the ArdunioISP setup: http://highlowtech.org/?p=1695.

Program starts to run when 5vdc power is applied to the ATtiny85.
