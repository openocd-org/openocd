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
<a href="index.tcl" style="font-weight: bold;">Target Status</a>
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
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1_selected.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="index.tcl" style="font-weight: bold;">Config Target</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="flashinfo.tcl">Flash</a>
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
<H1>OpenOCD debugger</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">

			
			<table>
				<tr><td style="height:10px;width:535px;">&nbsp</td></tr>
				<tr><td style="height:1px;width:535px;background-color:#a2c5d1;"></td></tr>
				<tr><td style="height:5px;width:535px;">&nbsp</td></tr>
			</table>

			<H1>Target Status</H1>

			<table>
				<tr>
					<td class="fontbigger">
						}

							set form_address [formfetch form_address]
							set form_action [formfetch form_action]
							
							if {[string compare $form_action "Halt"]==0} {
								append console [encode [capture_catch "halt"]]
							}
							if {[string compare $form_action "Resume"]==0} {
								append console [encode [capture_catch "resume"]]
							}
							  
							if {[string compare $form_action "Reset and run"]==0} {
								append console [encode [capture_catch "reset run"]]
							}
							
							if {[string compare $form_action "Power on"]==0} {
								append console [encode [capture_catch "power on"]]
							}
							if {[string compare $form_action "Power off"]==0} {
								append console [encode [capture_catch "power off"]]
							}
						
append buffer {
					
						}
append console [encode [capture_catch poll]]
append buffer {
					</td>
				</tr>
			</table>

			<form action="index.tcl" method="post"> 
				<table><tr>
					<td><input type="submit" name="form_action" value="Reset and run"></td>
					<td class="buttonspacesmall"></td><td><input type="submit" name="form_action" value="Halt"></td>
					<td class="buttonspacesmall"></td><td><input type="submit" name="form_action" value="Resume"></td>
					<td style="width:50px;"></td><td><input type="submit" name="form_action" value="Power on"></td>
					<td class="buttonspacesmall"></td><td><input type="submit" name="form_action" value="Power off"></td>
				</tr></table>

				<br>						
				<br>						
										
				<p>
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
<a class="openocd" href="index.tcl?toggle_details=1">
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
				
					<p>Target status shows that status of the connected target. </p> 
					<p><b>Current target</b> - selected target configuration. <br>
					<p><b>Startup</b> - whether or not the target script ran to completion. Note
					that even if the target is disconnected, powered down or unresponsive, the
					startup script will still run to completion. Startup - OK does not mean
					that the target is fully operational, simply that the configuration script
					did not contain syntax errors for instance. 
					See log for details. <br>
					<p><b>Target power</b> - Detects power on target. <br>
					If the JTAG cable is not connected, or the target has no power, then no target power will be detected.</p>
					<p>Type "help power" in telnet for command to control power relay.</p>
				
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

