# converted to .tcl by html2tcl.tcl
set buffer ""
append buffer {
	
	

		
		
		
		


		


		



		

		
		

		




		



		


		


		

		


		



		

		
		
		
		
		
		

		<html xmlns="http://www.w3.org/TR/REC-html40">
<head>
<title>Zylin ZY1000 JTAG debugger</title>
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
<img src="menu_cuts/logo_top.png" style="border:0px;"/>
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
<a href="zy1000.tcl" style="">Set IP Address</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab_selected.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="upgrade.tcl" style="font-weight: bold;">ZY1000 Firmware</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="editfile.tcl" style="">Edit File</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="support.tcl" style="">Support Request</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="log.tcl#tail" style="">View Tail of Log</a>
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
<a href="/ram/cgi/index.tcl">Config Target</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/flashinfo.tcl">Flash</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/browsemem.tcl">Memory</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/openocd.tcl">OpenOCD</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1_selected.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/zy1000.tcl" style="font-weight: bold;">Setup ZY1000</a>
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
<H1>Upgrade ZY1000 Firmware</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">
			
			}

				set form_action [formfetch form_action];
				set form_filecontent [formfetch form_filecontent];
				
				append buffer {<form enctype="multipart/form-data" action="upgrade.tcl" method="post">}
				if {[string compare $form_action "Upload"]==0} {
					
					if [string match ZylinPhiFirmware* $form_filecontent]==1 {
						set form_filename /config/firmware.phi 
						puts "Writing firmware to $form_filename"
						set fp [aio.open $form_filename w];
						$fp puts -nonewline $form_filecontent
						$fp close
						puts "Done writing firmware to $form_filename"
						append buffer "<br><div style='font-size:14px;'>Upgraded Zylin JTAG, rebooting (wait ca. 30 seconds)...</div><br>"
						reboot
					} else {
						append buffer "<br>Not a valid Zylin JTAG firmware file.<br>"
					}
				} elseif {[string compare $form_action "Restore factory settings"]==0} {
					proc rmdir { dir } {
						set entries {}
						catch {set entries [ls $dir]} err
						foreach { entry } $entries {
							rmdir $dir/$entry
							rm $dir/$entry
						}
					}				
					rmdir /config/settings
					reboot
					append buffer "Restoring factory settings (wait ca. 30 seconds)...<br>"
				} else {
					append buffer {<div style="font-size:14px;">Upgrade Zylin JTAG firmware + reboot. </div><p>}
					append buffer {<input type="file" name="form_filecontent"> <br>}
					append buffer {<table><tr><td style="height:15px;width:535px;">&nbsp</td></tr><tr><td style="height:1px;width:535px;background-color:#a2c5d1;"></td></tr><tr><td style="height:15px;width:535px;">&nbsp</td></tr></table>}
					append buffer {<input type="submit" name="form_action" value="Upload" ><br> }
					append buffer {<input type="submit" name="form_action" value="Restore factory settings" ><br> }
					append buffer {</form> }
				}
			
append buffer {
			
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
<a class="openocd" href="/ram/cgi/upgrade.tcl?toggle_details=1">
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
<a target="_blank" href="http://www.zylin.com/zy1000/ZY1000_Quick_Start_Guide.pdf">Quick Start Manual</a>
<br/>
<a target="_blank" href="http://www.zylin.com/zy1000/openocd.pdf">OpenOCD Manual</a>
<br/>
<a target="_blank" href="http://www.zylin.com/zy1000_contact.html">Contact Zylin AS</a>
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
				
					Upload new firmware file to ZY1000, verify that it is a valid ZY1000 firmware
					file, and upgrade flash.
					<p/>
					Restore factory settings restores all factory settings except the TCP/IP settings.
					<p/>
					The current version of the firmware is visible at the top of the web page.
					<p/>
					ZY1000 firmware can also be upgraded using serial port YModem upload.
					<p/>
					Check for new firmware at <a target="_blank" href="http://www.zylin.com/zy1000.html">http://www.zylin.com/zy1000.html<a> 
				  
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
<tr>
<td/>
<td>
<img border="0" src="menu_cuts/logo_bottom.png"/>
</td>
</tr>
</table>
</body>
</html>
		

		


		



		

		


		


		
	
	
}

start_chunked "html"
write_chunked $buffer
end_chunked

