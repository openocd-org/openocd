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
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab_selected.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="flash.tcl" style="font-weight: bold;">Program / Verify</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="production.tcl" style="">Production</a>
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
<H1>Program / Verify Flash</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">
			
			}

			
			set form_offset [formfetch form_offset]
			set form_action [formfetch form_action]
			set form_type [formfetch form_type]
			
			
			set post ""
			catch {set post $post_data} err
			
			if {[string compare $form_offset ""]==0} {
				set form_offset 0
			}
			if {[string compare $form_type ""]==0} {
				set form_type ""
			}
			
			
append buffer {<code style="white-space: nowrap;">}

			
			set data ""
			append buffer {<form enctype="multipart/form-data" action="flash.tcl" method="post">}
			
			set action_reset [expr {[string length $form_action]!=0}] 
			set action_flash [expr {[string compare $form_action "Flash"]==0 || [string compare $form_action "Flash and verify"]==0}] 
			set action_verify [expr {[string compare $form_action "Verify"]==0 || [string compare $form_action "Flash and verify"]==0}]
			
			if {$action_reset} {
				append console [encode [capture_catch "reset init"]]
			}
			
append buffer {
			</code>}

				
			append buffer {<table>}
			append buffer {<tr><td class="formtext">File</td><td><input type="file" name="form_filecontent"></td></tr>}
			append buffer "<tr><td class=\"formtext\" >Offset</td><td><input type=\"text\" name=\"form_offset\" value=\"$form_offset\"></td></tr>"
			
			
append buffer {
			<tr><td class="formtext" style="padding-top:1px;">Type</td><td>
			<select name="form_type">
			  		<option
			   }
if {[string compare $form_type ""]==0} { append buffer {selected="selected"} }  
append buffer {
			  		value ="">auto</option>
			  <option 
			   }
if {[string compare $form_type "elf"]==0} { append buffer {selected="selected"} }  
append buffer {
			  value ="elf">elf</option>
			  <option 
			   }
if {[string compare $form_type "bin"]==0} { append buffer {selected="selected"} }  
append buffer {
			  value ="bin">binary</option>
			  <option 
			   }
if {[string compare $form_type "ihex"]==0} { append buffer {selected="selected"} }  
append buffer {
			  value ="ihex">ihex</option>
			  <!-- broken <option value ="s19">s19</option> -->
			</select>
			</td>
			
			</tr>
			
			
			</table>
			
				<table>
					<tr><td style="height:15px;width:535px;">&nbsp</td></tr>
					<tr><td style="height:1px;width:535px;background-color:#a2c5d1;"></td></tr>
					<tr><td style="height:15px;width:535px;">&nbsp</td></tr>
				</table>
			
			<table><tr>
				<td><input type="submit" name="form_action" value="Flash" ></td>
				<td class="buttonspacesmall"></td><td><input type="submit" name="form_action" value="Flash and verify" ></td>
				<td class="buttonspacesmall"></td><td><input type="submit" name="form_action" value="Verify" ></td>
			</tr></table>
		
			<p>
			}

			
			if {$action_flash||$action_verify} {
				catch {writeform form_filecontent $upload_filename} result
				append console [encode $result]
			}
			append buffer "<br>"
			if {$action_flash} {
				append console [encode [capture_catch "halt"]]
				append buffer "<b>"
				if {[catch {capture_catch {eval "flash write_image erase $upload_filename $form_offset $form_type"}} result]} {
					append buffer "Flash write failed<br>"
					append console [encode $result]
				} else {
					append buffer [encode $result]
					append buffer "Flash write succeed<br>"
				}
				append buffer "</b>"
			}
			if {$action_verify} {
				append console [encode [capture_catch "halt"]]
				append buffer "<b>"
				if {[catch {capture_catch {eval "verify_image $upload_filename $form_offset $form_type"}} result]} {
					append buffer "Verify failed<br>"
					append console [encode $result]
				} else {
					append buffer [encode $result]
					append buffer "Verify succeed<br>"
				}
				append buffer "</b>"
			}
			
append buffer {		
				
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
<a class="openocd" href="flash.tcl?toggle_details=1">
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
				
				<p>Program and/or verify the flash on your target.</p>
				<p><b>Flash</b> - Halt CPU, automatically erase flash if required and program flash with image.</p>
				<p><b>Flash and verify</b> - Programs the flash and verifies the programmed flash content is correct.</p>
				<p><b>Verify</b> - Halt CPU and verify image in flash or RAM.</p>
				<p><b>Offset</b> - This value is added to the address of the image.<br> 
					Binary images start at address 0 by default, whereas elf and ihex have addresses encoded into the image.<br> 
					Typically 0 for elf/ihex and the address to	write the image to for binary files.</p>
					 
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

