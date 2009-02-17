# some dummy proc's to get things going for test purposes



proc ip {} {
return 10.0.0.55
}

proc start_chunked {a} {
	global httpdata
	global httpmime
	set httpmime $a
	set httpdata ""
}

proc write_chunked {a} {
	global httpdata
	append httpdata $a
}

proc end_chunked {} {
}



#proc formfetch {a} {
#	global httppostdata
	#catch { 
#	echo "$a=$httppostdata($a)"
	#return $httppostdata($a) 
	#}
#	
	#return ""  
#}




proc tohex {a} {
   set r ""
   while 1 {

      set rem [expr $a%16]
      set a [expr $a/16]
      set r [string index "0123456789abcdef" $rem]$r
      if ($a==0) then break
   }  
   return $r 
}

# encode text
proc encode {a} {
	return [string map {\n <br/> { } {&nbsp;} \t {&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;} > &gt; < &lt; / &#47;} $a]
}

#stubs that can be overriden to save between sessions
proc load_var {a} {
	global glob_var
	catch {
		return $glob_var($a)
	}
	return ""
}
#stubs that can be overriden to save between sessions
proc save_var {a b} {
	catch { 
	set glob_var($a) $b
	return ""
	} err
	set glob_var($a) ""
	return ""
}



proc to_textarea {a} {
	return [string map {& &#38; > &gt; < &lt; / &#47;} $a]
}	

proc from_textarea {a} {
	return [string map {&gt; > &lt; < &#38; & &#47; /} $a]
}
	
proc lunion {a b} {
	foreach e $a {
		set x($e) {}
	}
 	foreach e $b {
		if {![info exists x($e)]} {
    		lappend a $e
		}
	}
 	return $a
}
 

# encode text
proc encode {a} {
	return [string map {\n <br/> { } {&nbsp;} \t {&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;} > &gt; < &lt; / &#47;} $a]
}

proc first_flash_base {} {
	set t [lindex 0 [ocd_flash_banks]]
	return $t(base)
}
