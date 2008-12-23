# converted to .tcl by html2tcl.tcl
set buffer ""
append buffer {
	
	

		
		
		
		


		


		



		

		
		

		




		



		


		


		

		


		



		

		
		
		
		
		
		

		
		

		


		



		

		
		<html xmlns="http://www.w3.org/TR/REC-html40">
<head>
<title>Zylin ZY1000 JTAG debugger</title>
<meta charset="utf-8" content="text/html" http-equiv="Content-Type"/>
<link type="text/css" rel="stylesheet" href="/ram/cgi/zylweb.css"/>
</head>
}

				set console ""
				set upload_filename /ram/upload
			
append buffer {
<body style="margin:0px;">
<div style="width:974px;height:85px;">
<div style="float:left;position:relative;left:32px;width:478px;">
<a href="/">
<img src="/rom/menu_cuts/logo_top.png" style="border:0px;"/>
</a>
</div>
<div style="float:left;position:relative;height:26px; width:278px;left:122px;background-image:url('/rom/menu_cuts/top_right.png');">
<div style="position:relative;left:15px;top:4px;" class="textlight">
}
append buffer [capture zy1000_version]
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
<td style="width:140px;height:38px;background-image:url('/rom/menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="zy1000.tcl" style="">Set IP Address</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('/rom/menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="upgrade.tcl" style="">ZY1000 Firmware</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('/rom/menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="editfile.tcl" style="">Edit File</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('/rom/menu_cuts/v_tab_selected.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="support.tcl" style="font-weight: bold;">Support Request</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:38px;background-image:url('/rom/menu_cuts/v_tab.png');background-repeat: no-repeat;">
<div style="position:relative;left:10px;top:10px;font-weight:bold;">
<a href="log.tcl#tail" style="">View Tail of Log</a>
</div>
</td>
</tr>
<tr>
<td style="width:140px;height:35px;background-image:url('/rom/menu_cuts/v_1.png')"/>
</tr>
<tr>
<td style="width:140px;background-image:url('/rom/menu_cuts/v_2_tile.png')"/>
</tr>
<tr>
<td style="width:140px;height:140px;background-image:url('/rom/menu_cuts/v_3.png')"/>
</tr>
</table>
</td>
<td style="vertical-align:top;padding:0px;height:100%">
<table style="padding:0px;border-collapse:collapse;height:100%;">
<tr>
<td>
<table style="padding:0px;border-collapse:collapse;">
<tr>
<td style="width:103px;height:29px;background-image:url('/rom/menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/index.tcl">Config Target</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('/rom/menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/flashinfo.tcl">Flash</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('/rom/menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/browsemem.tcl">Memory</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('/rom/menu_cuts/h_tab1.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/openocd.tcl">OpenOCD</a>
</div>
</td>
<td style="width:103px;height:29px;background-image:url('/rom/menu_cuts/h_tab1_selected.png');background-repeat: no-repeat;">
<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
<a href="/ram/cgi/zy1000.tcl" style="font-weight: bold;">Setup ZY1000</a>
</div>
</td>
</tr>
</table>
</td>
</tr>
<tr>
<td style="height:30px;width:535px;background-image:url('/rom/menu_cuts/center_top.png');background-repeat: no-repeat;background-position:top right;" colspan="6">
<div style="width:500px;background-color:#ffffff;height:100%;">
								 			&nbsp;
							 			</div>
</td>
</tr>
<tr>
<td style="background-color:#ffffff;text-indent:30px;height:40px;" colspan="6">
<H1>Submit Support Request</H1>
</td>
</tr>
<tr style="height:100%;">
<td style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;" colspan="6">
			Before contacting Zylin, please submit a support request with relevant information. 
			}

			
			set form_config [load_config "target/[load_target]"]
			set support_id [string range [rand] 0 7]
			set form_log ""
			append form_log "Version: [capture "zy1000_version zy1000"]"
			append form_log "OpenOCD version: [capture "zy1000_version openocd"]"
			append form_log "Version date: [capture "zy1000_version date"]"
			append form_log [log]
			
append buffer {

			<form action="supportrequest.tcl" method="POST" target="_blank">
				<input TYPE="hidden" NAME="id" VALUE="}
append buffer $support_id
append buffer {">
				<input TYPE="hidden" NAME="success" VALUE="http://www.zylin.com/zy1000_support.html">
				<input size="50" name="subject" type="hidden" value="ZY1000 support request">
				<table cellspacing="5">
				<tr><td>Support ID</td><td>}
append buffer $support_id
append buffer {</td></tr>
				<tr><td>Contact person</td><td><input size="50" name="name" type="text"></td></tr>
				<tr><td>Phone</td><td><input size="50" name="phone" type="text"></td></tr>
				<tr><td>email</td><td><input size="50" name="email" type="text"></td></tr>
				<tr><td>MAC address</td><td><input size="50" name="serial" type="text" value="}
append buffer [mac]
append buffer {"></td></tr>
				</td></tr>
				</table>
				<p>
				Summary:
				<p>
				<input name="summary" size="50">
				<p>
				Description:
				<p>
				<textarea  style="overflow:auto;font-size:11px;"  name="description" cols="50" rows="4" type="textarea" wrap="off"></textarea>
				
				<p>				
				Log:<p>
				<textarea  style="overflow:auto;font-size:11px;"  name="log" cols="50" rows="5" type="textarea" wrap="off">}
append buffer $form_log
append buffer {</textarea>
				<p>				
				Config:<p>
				<textarea  style="overflow:auto;font-size:11px;"  name="config" cols="50" rows="5" type="textarea" wrap="off">}
append buffer $form_config
append buffer {</textarea>
				<p>
			</form>
			<p>
			<input value="Creates support request" type="submit"/></td></tr>

			
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
												 			append buffer {td style="background-image:url('/rom/menu_cuts/h_tab_free.png');width:110px;height:29px;background-repeat: no-repeat;background-position:top left;"}
												 			append buffer >
												 		}
												 	
append buffer {
<a class="openocd" href="/ram/cgi/support.tcl?toggle_details=1">
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
<td style="height:30px;background-image:url('/rom/menu_cuts/center_bottom.png');background-repeat: no-repeat;background-position:top right;" colspan="6">
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
<td style="width:103px;height:29px;background-image:url('/rom/menu_cuts/h_tab2_selected.png');background-repeat: no-repeat;">
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
<td style="height:10px;width:245px;background-image:url('/rom/menu_cuts/right_top_small.png');" colspan="3"/>
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
				
				Before contacting Zylin with questions, please fill in and submit this form
				and allow us time to review the information and answer by email if possible.
				<p/> 
				Note that you can see precisely what information is submitted to Zylin in the
				form: the log and your config files.
				
			</td>
</tr>
<tr>
<td style="height:30px;background-image:url('/rom/menu_cuts/right_bottom.png');" colspan="3">
							 			&nbsp;
							 		</td>
</tr>
</table>
</td>
</tr>
<tr>
<td/>
<td>
<img border="0" src="/rom/menu_cuts/logo_bottom.png"/>
</td>
<td style="padding-top:10px;padding-left:10px;margin-top:10px;" class="textlight">
							Zylin AS, Auglendsdalen 78, N-4017 Stavanger, Norway - www.zylin.com
						</td>
</tr>
</table>
</body>
</html>



		


		
	
	
}

start_chunked "html"
write_chunked $buffer
end_chunked

