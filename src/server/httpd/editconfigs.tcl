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
<a href="index.tcl" style="">Target Status</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="preconfig.tcl" style="">Select Target Config</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab_selected.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="editconfigs.tcl" style="font-weight: bold;">Edit Configurations</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="reload.tcl" style="">Reload Config Scripts</a>
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
<H1>Edit Target Configurations</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">
			<form action="editconfigs.tcl" method="post">
			}

				set form_edittext [formfetch form_edittext]
				set form_action [formfetch form_action]
				set form_filename [formfetch form_filename]
				set form_selected [formfetch form_selected] 
				
				if {[string compare $form_action "Load"]==0} {
					set form_filename $form_selected
				}
				
				if {[string compare $form_action "Delete"]==0} {
					capture_catch "rm /config/settings/$form_selected"
				}
				
				if {[string compare $form_action "Save"]==0} {
					save_var $form_filename [from_textarea $form_edittext] 
					append buffer "Wrote file $form_filename<br>"
				}
			
				set form_edittext ""
				
				# load original or script saved on disk.
				if {[string compare $form_action "Show default"]==0} {
					set form_edittext [load_file "/rom/$form_selected"]
					set form_filename $form_selected
				} else {
				    set form_edittext [load_config $form_filename]
				}
			
				set form_edittext_subst [to_textarea $form_edittext]
			
				
				proc prepend { val list } {
					set res ""				
			        foreach value $list {
			        	set t $val
			        	append t $value
			            lappend res $t
			        }
			        return $res
				 }				
				
				set files [prepend target/ [ls /rom/target]]
				set files [lunion $files [prepend event/ [ls /config/settings/event]]]
				set files [lunion $files [prepend target/ [ls /config/settings/target]]]
				set files [lsort $files]
				
				
append buffer {
				<table style="padding:0px;border-collapse:collapse;"><tr>
					<td style="padding-top:1px;"><select name="form_selected">
						}

							set foundTarget 0
							foreach i $files {
								
append buffer {
							  		<option 
							  		}

								  		if {[string compare $form_filename $i]==0} { 
											set foundTarget 1
									  		append buffer {selected="selected"} 
								  		}
								  	
append buffer {
						  		value="}
append buffer $i
append buffer {">}
append buffer $i
append buffer {</option>
								}

							}
							if {$foundTarget==0} {
								
append buffer {
							  		<option selected="selected" value="">Select target config</option>
								}

							}
						
append buffer {
					</select></td>
					<td class="buttonspacesmall">&nbsp</td>
					<td><input type="submit" value="Load" name="form_action"></td>
					<td class="buttonspacesmall">&nbsp</td>
					<td><input type="submit" value="Show default" name="form_action"></td>
					<td class="buttonspacesmall">&nbsp</td>
					<td><input type="submit" value="Delete" name="form_action"></td>
				</tr></table>
				<textarea  style="overflow:auto;"  rows="21" cols="65" name="form_edittext" wrap="off">}
append buffer $form_edittext_subst
append buffer {</textarea>
				<table style="padding:0px;border-collapse:collapse;"><tr>
				}

					append buffer {<td class="formtext">File</td><td><input type="text" name="form_filename" } "\n"
					append buffer "value=\"$form_filename\" ></td>\n"
					append buffer {<td class="buttonspacesmall">&nbsp</td><td><input type="submit" value="Save" name="form_action"></td><br>} "\n"
					append buffer {</tr></table>} "\n"
				
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
<a class="openocd" href="editconfigs.tcl?toggle_details=1">
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
				
				<p>Here you can edit predefined target configurations, restore predefined configurations to
				default state and create new target configurations.<p/>
				<p>Typically when creating a new target configuration, you would take an existing
				configuration that resembles the most your needs and modify it for your
				purposes and save it under a different name.</p>
				<p><b>Load</b> - Loads a configuration file into the editor.</p>
				<p><b>Show default</b> - Loads the firmware included version of the
				configuration file (if any), into the editor.<br>
				<b>Note</b> that the editor content is not saved.</p>
				<p><b>Delete</b> - Deletes a custom created configuration file.<br>
				<b>Note</b> that firmware included configuration files can not be deleted.</p>
				<p><b>Save</b> - Save the edited file under the a new or the same name.</p>
				
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

