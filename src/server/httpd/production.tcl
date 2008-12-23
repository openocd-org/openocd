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
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="flashinfo.tcl" style="">Info</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="erase.tcl" style="">Erase</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="flash.tcl" style="">Program / Verify</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab_selected.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="production.tcl" style="font-weight: bold;">Production</a>
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
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1_selected.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="flashinfo.tcl" style="font-weight: bold;">Flash</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="browsemem.tcl">Memory</a>
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
<H1>Production</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">
			}

				set form_action [formfetch form_action]
				set form_serialnumber [formfetch form_serialnumber]
				append buffer [production_info]
			
append buffer {
				
			<form enctype="multipart/form-data" action="production.tcl" method="post">
				<code style="white-space: nowrap;">
					}
	
						if {[string compare $form_action "Upload firmware"]==0} {
							set wrotedata [catch {writeform form_filecontent $upload_filename} result]  
							append buffer [encode $result]
							if {$wrotedata==0} {
								append buffer "<br>Running production procedure<p>"
								append buffer "<br>Reset and init: <br>"
								
								append console [encode [capture_catch {catch "production $upload_filename $form_serialnumber"}]]
							}
						}
						if {[string compare $form_action "Test"]==0} {
							append buffer "<br>Running production test. Output from first 10 seconds printed below. <p>"
							
							append console [encode [capture_catch {catch production_test}]]
						}
						if {[string compare $form_action "Power on"]==0} {
							append console [encode [capture_catch "power on"]]
						}
						if {[string compare $form_action "Power off"]==0} {
							append console [encode [capture_catch "power off"]]
						}
					
append buffer {
				</code>
				}

					append buffer {<p class="formtext">Firmware file(raw binary) <input type="file" name="form_filecontent"><p>}
					append buffer {<p class="formtext">Serial number <input type="text" name="form_serialnumber"><p>}
				
append buffer {
				
				<table>
					<tr><td style="height:15px;width:535px;">&nbsp</td></tr>
					<tr><td style="height:1px;width:535px;background-color:#a2c5d1;"></td></tr>
					<tr><td style="height:15px;width:535px;">&nbsp</td></tr>
				</table>
			
				<table><tr>
					<td><input type="submit" name="form_action" value="Upload firmware" ></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Test"></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Power on"></td>
					<td class="buttonspacesmall">&nbsp</td><td><input type="submit" name="form_action" value="Power off">
				</tr></table>
			</form>
			
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
<a class="openocd" href="production.tcl?toggle_details=1">
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
				
				The target script can implement the "production", "production_info" and "production_test" tcl proc's. These procedures
				are used on this page. There are default implementations that do nothing.
				
				<p><b>Upload firmware</b> - Power cycle target, reset target and program raw binary file to flash bank 0, offset 0 and verify flash programming. Leave target powered on.</p>
				<p><b>Test</b> -  Power up target, run 10 second target test. Output is provided via the DCC output channel. </p>
				<p><b>Power on</b> - Power on target.</p>
				<p><b>Power off</b> - Power off target.</p>
				<p><b>Serial number</b> - A target script can use this string in the production procedure. Type "help production" for more info.</p>
					 
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

