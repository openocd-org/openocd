# converted to .tcl by html2tcl.tcl
set buffer ""
append buffer {
	
	

		
		
		


		



		
		

		




		



		


		


		<html xmlns="http://www.w3.org/TR/REC-html40">
<head>
<title>OpenOCD debugger</title>
<meta charset="utf-8" content="text/html" http-equiv="Content-Type"/>
<link type="text/css" rel="stylesheet" href="menuweb.css"/>
</head>
}

				set console ""
				set upload_filename /ram/upload
			
append buffer {
<body style="margin:0px;">
<div style="width:974px;height:85px;">
<div style="float:left;position:relative;left:32px;width:478px;">
<a href="/">
							OpenOCD
						</a>
</div>
<div style="float:left;position:relative;height:26px; width:278px;left:122px;background-image:url('menu_cuts/top_right.png');">
<div style="position:relative;left:15px;top:4px;" class="textlight">
}
append buffer [capture version]
append buffer {
</div>
</div>
</div>
<table style="padding:0px;border-collapse:collapse;">
<tr>
<td style="width:33px;">
<div style="width:20px;height:510px;">
								&nbsp;
							</div>
</td>
<td style="vertical-align:top;height:100%;width:140px;padding:0px;">
<table style="padding:0px;border-collapse:collapse;height:100%;width:140px;">
<tr style="height:59px;">
<td/>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab_selected.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="browsemem.tcl" style="font-weight: bold;">Browse / Edit</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="downloadmem.tcl" style="">Download</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:35px;background-image:url('menu_cuts/v_1.png')"/>
</tr>
<tr>
<td style="width:140px;background-image:url('menu_cuts/v_2_tile.png')"/>
</tr>
<tr>
<td style="width:140px;height:140px;background-image:url('menu_cuts/v_3.png')"/>
</tr>
</table>
</td>
<td style="vertical-align:top;padding:0px;height:100%">
<table style="padding:0px;border-collapse:collapse;height:100%;">
<tr>
<td>
<table style="padding:0px;border-collapse:collapse;">
<tr>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="index.tcl">Config Target</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="flashinfo.tcl">Flash</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1_selected.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="browsemem.tcl" style="font-weight: bold;">Memory</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="openocd.tcl">OpenOCD</a>
</div>
</td>
</tr>
</table>
</td>
</tr>
<tr>
<td style="height:30px;width:535px;background-image:url('menu_cuts/center_top.png');background-repeat: no-repeat;background-position:top right;" colspan="6">
<div style="width:500px;background-color:#ffffff;height:100%;">
								 			&nbsp;
							 			</div>
</td>
</tr>
<tr>
<td style="background-color:#ffffff;text-indent:30px;height:40px;" colspan="6">
<H1>Browse / Edit Memory</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">


			
			}

			
			set form_address [formfetch form_address]
			set form_length [formfetch form_length]
			set form_type [formfetch form_type]
			set form_action [formfetch form_action]
			set form_value [formfetch form_value]
			
			if {[string compare $form_length ""]==0} {
				set form_length 0
			}  
			if {$form_length<=0} {
				set form_length 0x80
			} 
			if {$form_length>0x1000} {
				set form_length 0x1000
			} 
			
			if {[string compare $form_type ""]==0} {
				set form_type mdw
			}
			
			if {[string compare $form_type "mdw"]==0} {
				set wordsize 4
				set modify_cmd mww 
			}
			if {[string compare $form_type "mdh"]==0} {
				set wordsize 2
				set modify_cmd mwh 
			}
			if {[string compare $form_type "mdb"]==0} {
				set wordsize 1
				set modify_cmd mwb 
			}
			
			
			
			
			if {[string compare $form_address ""]!=0} {
				if {[string compare $form_action "Previous"]==0} {
					# Kludge! Work around problems parsing hex in Jim Tcl expressions
					incr form_address ; set form_address [expr $form_address-1]
					if {$form_address-$form_length>0} {
						set form_address "0x[tohex [expr $form_address-$form_length]]"
					} else {
						set form_address "0x0"
					}
				}  
				if {[string compare $form_action "Next"]==0} {
					# Kludge! Work around problems parsing hex in Jim Tcl expressions
					incr form_address ; set form_address [expr $form_address-1]
					set form_address "0x[tohex [expr $form_address+$form_length]]"
				}  
				if {[string compare $form_action "Modify"]==0} {
					append console [capture_catch "$modify_cmd $form_address $form_value"]
				}  
				if {[string compare $form_action "Fill"]==0} {
					append console [capture_catch "$modify_cmd $form_address $form_value $form_length"]
				}  
			}
			
			
			
append buffer {
			
			<form action="browsemem.tcl" method="post"> 
				<table>
				<tr><td class="formtext">Address</td><td><input type="text" name="form_address" value="}
append buffer $form_address
append buffer {"></td></tr>
				<tr><td class="formtext">Length</td><td><input type="text" name="form_length" value="}
append buffer "0x[tohex $form_length]"
append buffer {"></td></tr>
				<tr><td class="formtext">Value</td><td><input type="text" name="form_value" value="}
append buffer $form_value
append buffer {"></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Modify"></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Fill"></td></tr>
				<tr><td class="formtext">Type</td><td style="padding-top:1px;">
				<select name="form_type">
				  <option 
				    }
if {[string compare $form_type "mdb"]==0} { append buffer {selected="selected"} }  
append buffer { value ="mdb">8 bit
				  </option>
				  <option 
				   }
if {[string compare $form_type "mdh"]==0} { append buffer {selected="selected"} }  
append buffer { value ="mdh">16 bit
				  </option>
			  		<option
					   }
if {[string compare $form_type "mdw"]==0} { append buffer {selected="selected"} }  
append buffer {value ="mdw">32 bit
				  	</option>
				</select>
				
				</td></tr>
				</table>
				<table>
					<tr><td style="height:15px;width:535px;">&nbsp</td></tr>
					<tr><td style="height:1px;width:535px;background-color:#a2c5d1;"></td></tr>
					<tr><td style="height:15px;width:535px;">&nbsp</td></tr>
				</table>
			
				<table><tr>
					<td><input type="submit" name="form_action" value="Refresh"></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Previous" ></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Next" ></td>
				</tr></table>
				<br>
				
			</form>
			<p>
			<div class="fontbigger">Memory:</div><p>
			<code style="white-space: nowrap; font-size:11px;font:courier new;">
				}

				if {[string compare $form_address ""]!=0} {
					append console [encode [capture_catch halt]]
					append buffer [encode [capture_catch "$form_type $form_address [expr $form_length]"]]
				} 
				
append buffer {
			</code>


			

			
			</td>
</tr>
}

							 		
								 	set toggle_details [formfetch toggle_details]
								 	if {[string length $toggle_details]==0} {
								 		set toggle_details 0
								 	}
								 	set show_details [load_var show_details]
								 	if {[string length $show_details]==0} {
								 		set show_details 0
								 	}
								 	if {$toggle_details==1} {
								 		set show_details [expr 1-$show_details]
								 		save_var show_details $show_details
								 	}
								 	
							 		if {[string length $console]!=0} {
							 			
append buffer {
<tr style="height:100%;">
<td style="height:100%;background-color:red;" colspan="6">
<table style="padding:0px;border-collapse:collapse;background-color:#ffffff;width:100%" class="textgray">
<td style="width:25px;">&nbsp;</td>
}

												 		if {$show_details==1} {
												 			append buffer <
												 			append buffer {td style="background-color:#dddddd;padding-left:5px;padding-right:5px;padding-top:3px;padding-bottom:3px;"}
												 			append buffer >
												 		} else {
												 			append buffer <
												 			append buffer {td style="background-image:url('menu_cuts/h_tab_free.png');width:110px;height:29px;background-repeat: no-repeat;background-position:top left;"}
												 			append buffer >
												 		}
												 	
append buffer {
<a class="openocd" href="browsemem.tcl?toggle_details=1">
}

															if {$show_details==1} {
																append buffer "Hide details"
													 			append buffer <br/>
															} else {
																append buffer {<div style="position:relative;top:7px;text-align:center;">}
																append buffer "Show details"
																append buffer {</div>}
															}
															
append buffer {
</a>
}

													 		if {$show_details==1} {
													 			append buffer $console
													 		}
													 	
append buffer {</td>}

													 	if {$show_details!=1} {
													 		append buffer {<td>&nbsp;</td>}
													 	}
													 
append buffer {
<td style="width:25px;">&nbsp;</td>
</table>
</td>
</tr>
}

									 }
								
append buffer {
<tr>
<td style="height:30px;background-image:url('menu_cuts/center_bottom.png');background-repeat: no-repeat;background-position:top right;" colspan="6">
<div style="width:500px;background-color:#ffffff;height:100%;">
								 			&nbsp;
							 			</div>
</td>
</tr>
</table>
</td>
<td style="width:6px;"/>
<td style="width:245px;height:100%">
<table style="padding:0px;border-collapse:collapse;height:100%;">
<tr>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab2_selected.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;;font-weight:bold;text-align:center;width:100px;" class="textgray">
										    Documentation
										 </div>
</td>
<td width="40px">
								 		&nbsp;
								 	</td>
<td/>
</tr>
<tr>
<td style="height:10px;width:245px;background-image:url('menu_cuts/right_top_small.png');" colspan="3"/>
</tr>
<tr>
<td style="background-color:#d8d7d7;width:245px;padding-left:10px;padding-buttom:10px;line-height:17px;" colspan="3">
<a target="_blank" href="http://openocd.berlios.de/doc/openocd.pdf">OpenOCD Manual</a>
<br/>
</td>
</tr>
<tr>
<td style="background-color:#d8d7d7;height:15px;" colspan="3"/>
</tr>
<tr>
<td colspan="3">
<table style="padding:0px;border-collapse:collapse;">
<td style="background-color:#d8d7d7;width:10px;height:1px"/>
<td style="background-color:#999999;width:225px; height:1px;"/>
<td style="background-color:#d8d7d7;width:10px;height:1px"/>
</table>
</td>
</tr>
<tr>
<td style="background-color:#d8d7d7;height:15px;" colspan="3"/>
</tr>
<tr style="height:100%;">
<td style="height:100%;background-color:#d8d7d7;padding-left:10px;padding-right:10px;" colspan="3" class="textgray">
				
				<p>Browse and edit target memory.<br>
				   Length is in bytes, maximum 4096 bytes.</p> 
				<p>An error message is shown when trying to browse or edit memory which cases a CPU fault.</p>
				<p>CPU will be halted if required.</p>
				<p><b>Modify</b> - Will modify only one byte, half-word or word starting at Address.</p>
				<p><b>Fill</b> - Will fill the specified region with the specified value.</p>
				<p><b>Refresh</b> - Display the content of the specified memory area.</p>
					
			</td>
</tr>
<tr>
<td style="height:30px;background-image:url('menu_cuts/right_bottom.png');" colspan="3">
							 			&nbsp;
							 		</td>
</tr>
</table>
</td>
</tr>
</table>
</body>
</html>

		


		



		

		
		
		
		


		


		


		


		
	
	
}

start_chunked "html"
write_chunked $buffer
end_chunked

