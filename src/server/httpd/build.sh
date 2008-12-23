set e
java -classpath ../../../../zy1000/build/xalan.jar\;. Stylizer menu.xsl menu.xml .
find . -regex ".*\.tcl" -type f -exec sh html2tcl.sh {} {} \;
echo "Copy .tcl files to /usr/local/lib/openocd/httpd/"
cp *.tcl /usr/local/lib/openocd/httpd/