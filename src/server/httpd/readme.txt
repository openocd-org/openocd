work in progress... stay tuned....


1. To build .tcl pages. This will convert menu.xml and menu.xsl into
lots of .html pages w/embedded tcl, which are then inverted into
tcl with embedded html.

sh build.sh

xalan.jar can be gotten from apache.org. 


2. libmicrohttpd is a bit tricky to build under Cygwin:

https://gnunet.org/mantis/view.php?id=1440

3. To test:

../openocd/configure --enable-httpd --enable-dummy  --enable-ioutil
make
make install
openocd  -f httpd/httpd.tcl -c "interface dummy" -f target/at91eb40a.cfg

4. Point browser to: http://localhost:8888
