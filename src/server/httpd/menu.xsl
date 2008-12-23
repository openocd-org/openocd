<?xml version="1.0"?>
<!DOCTYPE xsl:stylesheet [<!ENTITY nbsp "&#160;">]>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns="http://www.w3.org/TR/REC-html40" version="1.0">
	<xsl:output method="html" version="4.0" indent="yes" encoding="UTF-8"
	
	 media-type="text/plain; charset=UTF-8"/>

	<xsl:param name="pagetogenerate" select="UNDEFINED"/>
	<xsl:template match="page[outfile!=$pagetogenerate]">
	</xsl:template>

	<xsl:template match="page[outfile=$pagetogenerate]">
		<xsl:variable name="Xlevel2parent" select="level2parent"/>
		<xsl:variable name="Xlevel3parent" select="level3parent"/>

		<html>
			<head>
				<title>OpenOCD debugger</title>
				<meta http-equiv="Content-Type" content="text/html"  charset="utf-8"/>
				<link href="menuweb.css" rel="stylesheet" type="text/css"/>				
				
			</head>
	
			
			<tcl>
				set console ""
				set upload_filename /ram/upload
			</tcl>
			
			<body style="margin:0px;">
				<div style="width:974px;height:85px;">
					<div style="float:left;position:relative;left:32px;width:478px;">
						<a href="/">
							OpenOCD
						</a>
					</div>
					<div style="float:left;position:relative;height:26px; width:278px;left:122px;background-image:url('menu_cuts/top_right.png');">
						<div class="textlight" style="position:relative;left:15px;top:4px;">
							<tcl>append buffer [capture version]</tcl>
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
						<!-- level 2 menu bar on left -->
						<td style="vertical-align:top;height:100%;width:140px;padding:0px;">
							<table style="padding:0px;border-collapse:collapse;height:100%;width:140px;">
								<tr style="height:59px;">
									<td></td>
								</tr>
								<xsl:for-each select="parent::language/page[outfile = $Xlevel2parent]/level2menu">
									<tr>
										<td>
											<xsl:choose>
												<xsl:when test="contains(@href, $pagetogenerate)">
													<xsl:attribute name="style">width:140px;height:38px;background-image:url('menu_cuts/v_tab_selected.png');background-repeat: no-repeat;</xsl:attribute>
												</xsl:when>
												<xsl:otherwise>
													<xsl:attribute name="style">width:140px;height:38px;background-image:url('menu_cuts/v_tab.png');background-repeat: no-repeat;</xsl:attribute>
												</xsl:otherwise>
											</xsl:choose>
											<div style="position:relative;left:10px;top:10px;font-weight:bold;">
												<a>
													<xsl:attribute name="href">
														<xsl:value-of select="@href"/>
													</xsl:attribute>
													<xsl:choose>
														<xsl:when test="(@href = $pagetogenerate)">
															<xsl:attribute name="style">font-weight: bold;</xsl:attribute>
														</xsl:when>
														<xsl:otherwise>
															<xsl:choose>
																<xsl:when test="(@href = $Xlevel3parent)">
																	<xsl:attribute name="style">font-weight: bold;</xsl:attribute>
																</xsl:when>
																<xsl:otherwise>
																	<xsl:attribute name="style"></xsl:attribute>
																</xsl:otherwise>
															</xsl:choose>
														</xsl:otherwise>
													</xsl:choose>
													<xsl:value-of select="@title"/>
												</a>
											</div>
										</td>
									</tr>
								</xsl:for-each>
								<tr>
									<td style="width:140px;height:35px;background-image:url('menu_cuts/v_1.png')">
									
									</td>
								</tr>
								<tr>
									<td style="width:140px;background-image:url('menu_cuts/v_2_tile.png')">
									
									</td>
								</tr>
								<tr>
									<td style="width:140px;height:140px;background-image:url('menu_cuts/v_3.png')">
									
									</td>
								</tr>
							</table>
						</td>
						<!-- top level menu -->
						<td style="vertical-align:top;padding:0px;height:100%">
							<table style="padding:0px;border-collapse:collapse;height:100%;">
								<tr>
									<td>
										<table style="padding:0px;border-collapse:collapse;">
											<tr>
												<xsl:for-each select="parent::language/page">
													<xsl:if test="menutext">
														<td>
															<xsl:choose>
																<xsl:when test="(outfile = $pagetogenerate) or (outfile = $Xlevel2parent)">
																	<xsl:attribute name="style">width:103px;height:29px;background-image:url('menu_cuts/h_tab1_selected.png');background-repeat: no-repeat;</xsl:attribute>
																</xsl:when>
																<xsl:otherwise>
																	<xsl:attribute name="style">width:103px;height:29px;background-image:url('menu_cuts/h_tab1.png');background-repeat: no-repeat;</xsl:attribute>
																</xsl:otherwise>
															</xsl:choose>
															<div style="position:relative;top:7px;font-weight:bold;text-align:center;width:100px;">
																<a>
																	<xsl:attribute name="href"><xsl:value-of select="menulink"/></xsl:attribute>
																	<xsl:if test="(outfile = $pagetogenerate)">
																		<xsl:attribute name="style">font-weight: bold;</xsl:attribute>
																	</xsl:if>
																	<xsl:if test="(outfile = $Xlevel2parent)">
																		<xsl:attribute name="style">font-weight: bold;</xsl:attribute>
																	</xsl:if>
																	<xsl:value-of select="menutext"/>
																</a>
															 </div>
														 </td>
													</xsl:if>
												</xsl:for-each>
											</tr>
										</table>
								 	</td>
								</tr>			
							 	<tr>
							 		<td colspan="6" style="height:30px;width:535px;background-image:url('menu_cuts/center_top.png');background-repeat: no-repeat;background-position:top right;">
							 			<div style="width:500px;background-color:#ffffff;height:100%;">
								 			&nbsp;
							 			</div>
							 		</td>
							 	</tr>
							 	<tr>
							 		<td colspan="6" style="background-color:#ffffff;text-indent:30px;height:40px;">
							 			<H1><xsl:value-of select="pageheading"/></H1>
							 		</td>
							 	</tr>
							 	<tr style="height:100%;">
							 		<td colspan="6" style="background-color:#ffffff;padding-left:30px;padding-right:30px;width=535px;height:100%;">
										<xsl:value-of disable-output-escaping="yes" select="pagetext/markup_code"/>
							 		</td>
							 	</tr>
							 	<tcl>
							 		<!-- This is the output from any OpenOCD commands -->
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
							 			</tcl>
									 	<tr style="height:100%;">
									 		<td colspan="6" style="height:100%;background-color:red;">
												<table class="textgray" style="padding:0px;border-collapse:collapse;background-color:#ffffff;width:100%">
											 		<td style="width:25px;">&nbsp;</td>
											 		<tcl>
												 		if {$show_details==1} {
												 			append buffer <xsl:text disable-output-escaping="yes"><![CDATA[<]]></xsl:text>
												 			append buffer {td style="background-color:#dddddd;padding-left:5px;padding-right:5px;padding-top:3px;padding-bottom:3px;"}
												 			append buffer <xsl:text disable-output-escaping="yes"><![CDATA[>]]></xsl:text>
												 		} else {
												 			append buffer <xsl:text disable-output-escaping="yes"><![CDATA[<]]></xsl:text>
												 			append buffer {td style="background-image:url('menu_cuts/h_tab_free.png');width:110px;height:29px;background-repeat: no-repeat;background-position:top left;"}
												 			append buffer <xsl:text disable-output-escaping="yes"><![CDATA[>]]></xsl:text>
												 		}
												 	</tcl>
											 			<a class="openocd">
															<xsl:attribute name="href"><xsl:value-of select="$pagetogenerate"/>?toggle_details=1</xsl:attribute>
															<tcl>
															if {$show_details==1} {
																append buffer "Hide details"
													 			append buffer <br/>
															} else {
																append buffer {<div style="position:relative;top:7px;text-align:center;">}
																append buffer "Show details"
																append buffer {</div>}
															}
															</tcl>
											 			</a>
											 			<tcl>
													 		if {$show_details==1} {
													 			append buffer $console
													 		}
													 	</tcl>
													 <xsl:text disable-output-escaping="yes"><![CDATA[<]]></xsl:text>/td<xsl:text disable-output-escaping="yes"><![CDATA[>]]></xsl:text>
													 <tcl>
													 	if {$show_details!=1} {
													 		append buffer {<td>&nbsp;</td>}
													 	}
													 </tcl>
											 		<td style="width:25px;">&nbsp;</td>
											 	</table>
										 	</td>
										 </tr>
									 	<tcl>
									 }
								</tcl>
							 	<tr>
							 		<td colspan="6" style="height:30px;background-image:url('menu_cuts/center_bottom.png');background-repeat: no-repeat;background-position:top right;">
							 			<div style="width:500px;background-color:#ffffff;height:100%;">
								 			&nbsp;
							 			</div>
							 		</td>
							 	</tr>
						 	</table>
						</td>
						<td style="width:6px;">
						</td>
						<td style="width:245px;height:100%">
							<table style="padding:0px;border-collapse:collapse;height:100%;">
								<tr>
									<td style="width:103px;height:29px;background-image:url('menu_cuts/h_tab2_selected.png');background-repeat: no-repeat;">
										<div class="textgray" style="position:relative;top:7px;;font-weight:bold;text-align:center;width:100px;">
										    Documentation
										 </div>
								 	</td>
								 	<td width="40px">
								 		&nbsp;
								 	</td>		
								 	<td>
								 	</td>	
								</tr>
							 	<tr>
							 		<td colspan="3" style="height:10px;width:245px;background-image:url('menu_cuts/right_top_small.png');"></td>
							 	</tr>
							 	<tr>
							 		<td colspan="3" style="background-color:#d8d7d7;width:245px;padding-left:10px;padding-buttom:10px;line-height:17px;">
							 			<a href="http://openocd.berlios.de/doc/openocd.pdf" target="_blank">OpenOCD Manual</a><br/>
							 		</td>
							 	</tr>
							 	<tr><td colspan="3" style="background-color:#d8d7d7;height:15px;"></td></tr>
							 	<tr>
							 		<td colspan="3">
							 			<table style="padding:0px;border-collapse:collapse;">
							 				<td style="background-color:#d8d7d7;width:10px;height:1px"></td>
							 				<td style="background-color:#999999;width:225px; height:1px;"></td>
							 				<td style="background-color:#d8d7d7;width:10px;height:1px"></td>
							 			</table>
							 		</td>
							 	</tr>
							 	<tr><td colspan="3" style="background-color:#d8d7d7;height:15px;"></td></tr>
							 	<tr style="height:100%;">
							 		<td class="textgray" colspan="3"  style="height:100%;background-color:#d8d7d7;padding-left:10px;padding-right:10px;">
										<xsl:choose>
											<xsl:when test="(pagetext/right_column)">
												<xsl:value-of disable-output-escaping="yes" select="pagetext/right_column"/>
											</xsl:when>
											<xsl:otherwise>
		
											</xsl:otherwise>
										</xsl:choose>
							 		</td>
							 	</tr>
							 	<tr>
							 		<td colspan="3" style="height:30px;background-image:url('menu_cuts/right_bottom.png');">
							 			&nbsp;
							 		</td>
							 	</tr>
							 </table>
				
						</td>
					</tr>
					
				</table>
			</body>
		</html>

	</xsl:template>
</xsl:stylesheet>
