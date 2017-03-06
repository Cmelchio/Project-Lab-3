<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.6.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="6" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="silabs">
<description>Silicon Laboratories&lt;p&gt;
C8051Fxxx family of mixed-signal microcontrollers integrates world-class analog,&lt;br&gt;
a high-speed pipelined 8051 CPU, ISP Flash Memory,&lt;br&gt;
and on-chip JTAG based debug in each device.&lt;br&gt;
The combination of configurable high-performance analog,&lt;br&gt;
100 MIPS 8051 core and in-system field programmability provides the user with complete design flexibility,&lt;br&gt;
improved time-to-market, superior system performance and greater end product differentiation.&lt;p&gt;

Source: http://www.silabs.com&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="8-PIN">
<description>&lt;b&gt;8-Pin PCB Land Pattern&lt;/b&gt;&lt;p&gt;
Source: si750.pdf</description>
<wire x1="-4" y1="3.55" x2="4" y2="3.55" width="0.2032" layer="51"/>
<wire x1="4" y1="3.55" x2="4" y2="-3.55" width="0.2032" layer="51"/>
<wire x1="4" y1="-3.55" x2="-4" y2="-3.55" width="0.2032" layer="51"/>
<wire x1="-4" y1="-3.55" x2="-4" y2="3.55" width="0.2032" layer="51"/>
<smd name="7" x="-2.8525" y="0" dx="1.545" dy="1.3" layer="1" stop="no" cream="no"/>
<smd name="8" x="2.8525" y="0" dx="1.545" dy="1.3" layer="1" stop="no" cream="no"/>
<smd name="5" x="0" y="2.1" dx="1.7" dy="2.15" layer="1" stop="no" cream="no"/>
<smd name="2" x="0" y="-2.1" dx="1.7" dy="2.15" layer="1" stop="no" cream="no"/>
<smd name="6" x="-2.54" y="2.1" dx="1.7" dy="2.15" layer="1" stop="no" cream="no"/>
<smd name="4" x="2.54" y="2.1" dx="1.7" dy="2.15" layer="1" stop="no" cream="no"/>
<smd name="1" x="-2.54" y="-2.1" dx="1.7" dy="2.15" layer="1" roundness="50" stop="no" cream="no"/>
<smd name="3" x="2.54" y="-2.1" dx="1.7" dy="2.15" layer="1" stop="no" cream="no"/>
<text x="-4.1216" y="3.7878" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.1216" y="-5.5658" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.325" y1="1.075" x2="-1.75" y2="3.1" layer="31"/>
<rectangle x1="-3.55" y1="-0.6" x2="-2.15" y2="0.6" layer="31"/>
<rectangle x1="-0.775" y1="1.075" x2="0.775" y2="3.1" layer="31"/>
<rectangle x1="1.75" y1="-3.1" x2="3.325" y2="-1.075" layer="31" rot="R180"/>
<rectangle x1="2.15" y1="-0.6" x2="3.55" y2="0.6" layer="31" rot="R180"/>
<rectangle x1="-0.775" y1="-3.1" x2="0.775" y2="-1.075" layer="31" rot="R180"/>
<rectangle x1="1.775" y1="1.075" x2="3.325" y2="3.1" layer="31" rot="R180"/>
<rectangle x1="-3.5" y1="0.925" x2="-1.575" y2="3.275" layer="29"/>
<rectangle x1="-3.725" y1="-0.75" x2="-1.975" y2="0.75" layer="29"/>
<rectangle x1="-0.96" y1="0.925" x2="0.965" y2="3.275" layer="29"/>
<rectangle x1="1.58" y1="0.925" x2="3.505" y2="3.275" layer="29"/>
<rectangle x1="1.575" y1="-3.275" x2="3.5" y2="-0.925" layer="29" rot="R180"/>
<rectangle x1="1.975" y1="-0.75" x2="3.725" y2="0.75" layer="29" rot="R180"/>
<rectangle x1="-0.965" y1="-3.275" x2="0.96" y2="-0.925" layer="29" rot="R180"/>
<polygon width="0.2032" layer="31">
<vertex x="-3.2" y="-2.775"/>
<vertex x="-3.2" y="-1.45" curve="-90"/>
<vertex x="-2.925" y="-1.2"/>
<vertex x="-2.125" y="-1.2" curve="-90"/>
<vertex x="-1.875" y="-1.45"/>
<vertex x="-1.875" y="-2.75" curve="-90"/>
<vertex x="-2.125" y="-3"/>
<vertex x="-2.975" y="-3" curve="-90"/>
</polygon>
<polygon width="0.2032" layer="29">
<vertex x="-3.375" y="-2.75"/>
<vertex x="-3.375" y="-1.45" curve="-90"/>
<vertex x="-2.975" y="-1.05"/>
<vertex x="-2.125" y="-1.05" curve="-89.971359"/>
<vertex x="-1.725" y="-1.45"/>
<vertex x="-1.725" y="-2.775" curve="-90"/>
<vertex x="-2.1" y="-3.15"/>
<vertex x="-2.95" y="-3.15" curve="-90"/>
</polygon>
</package>
</packages>
<symbols>
<symbol name="SI570">
<wire x1="-10.16" y1="10.16" x2="10.16" y2="10.16" width="0.254" layer="94"/>
<wire x1="10.16" y1="10.16" x2="10.16" y2="-10.16" width="0.254" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="-10.16" y2="-10.16" width="0.254" layer="94"/>
<wire x1="-10.16" y1="-10.16" x2="-10.16" y2="10.16" width="0.254" layer="94"/>
<text x="-10.16" y="11.43" size="1.778" layer="95">&gt;NAME</text>
<text x="-10.16" y="-12.7" size="1.778" layer="96">&gt;VALUE</text>
<pin name="OE" x="-12.7" y="2.54" length="short" direction="in"/>
<pin name="SDA" x="-12.7" y="-2.54" length="short" direction="in"/>
<pin name="SCL" x="-12.7" y="-7.62" length="short" direction="in"/>
<pin name="CLK-" x="12.7" y="-2.54" length="short" direction="out" rot="R180"/>
<pin name="CLK+" x="12.7" y="2.54" length="short" direction="out" rot="R180"/>
<pin name="GND" x="12.7" y="-7.62" length="short" direction="pwr" rot="R180"/>
<pin name="VDD" x="12.7" y="7.62" length="short" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SI570" prefix="IC">
<description>&lt;b&gt;ANY-RATE I2C PROGRAMMABLE XO/VCXO&lt;/b&gt;&lt;p&gt;
Source: si570.pdf</description>
<gates>
<gate name="G$1" symbol="SI570" x="0" y="0"/>
</gates>
<devices>
<device name="" package="8-PIN">
<connects>
<connect gate="G$1" pin="CLK+" pad="4"/>
<connect gate="G$1" pin="CLK-" pad="5"/>
<connect gate="G$1" pin="GND" pad="3"/>
<connect gate="G$1" pin="OE" pad="2"/>
<connect gate="G$1" pin="SCL" pad="8"/>
<connect gate="G$1" pin="SDA" pad="7"/>
<connect gate="G$1" pin="VDD" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="v-reg">
<description>&lt;b&gt;Voltage Regulators&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="78XXL">
<description>&lt;b&gt;VOLTAGE REGULATOR&lt;/b&gt;</description>
<wire x1="-5.207" y1="-1.27" x2="5.207" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.207" y1="14.605" x2="-5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-1.27" x2="5.207" y2="11.176" width="0.1524" layer="21"/>
<wire x1="5.207" y1="11.176" x2="4.318" y2="11.176" width="0.1524" layer="21"/>
<wire x1="4.318" y1="11.176" x2="4.318" y2="12.7" width="0.1524" layer="21"/>
<wire x1="4.318" y1="12.7" x2="5.207" y2="12.7" width="0.1524" layer="21"/>
<wire x1="5.207" y1="12.7" x2="5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-1.27" x2="-5.207" y2="11.176" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="11.176" x2="-4.318" y2="11.176" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="11.176" x2="-4.318" y2="12.7" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="12.7" x2="-5.207" y2="12.7" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="12.7" x2="-5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<circle x="0" y="11.176" radius="1.8034" width="0.1524" layer="21"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="42"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="43"/>
<pad name="IN" x="-2.54" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="GND" x="0" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="OUT" x="2.54" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<text x="-3.81" y="5.08" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.937" y="2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.445" y="7.874" size="0.9906" layer="21" ratio="10">A15,2mm</text>
<text x="-0.508" y="0" size="1.27" layer="51" ratio="10">-</text>
<text x="-3.048" y="0" size="1.27" layer="51" ratio="10">I</text>
<text x="2.032" y="0" size="1.27" layer="51" ratio="10">O</text>
<rectangle x1="1.905" y1="-2.159" x2="3.175" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-3.81" x2="3.175" y2="-2.159" layer="51"/>
<rectangle x1="-0.635" y1="-2.159" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="-3.175" y1="-2.159" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="-0.635" y1="-3.81" x2="0.635" y2="-2.159" layer="51"/>
<rectangle x1="-3.175" y1="-3.81" x2="-1.905" y2="-2.159" layer="51"/>
<hole x="0" y="11.176" drill="3.302"/>
</package>
</packages>
<symbols>
<symbol name="78XX">
<wire x1="-5.08" y1="-5.08" x2="5.08" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="5.08" y1="-5.08" x2="5.08" y2="2.54" width="0.4064" layer="94"/>
<wire x1="5.08" y1="2.54" x2="-5.08" y2="2.54" width="0.4064" layer="94"/>
<wire x1="-5.08" y1="2.54" x2="-5.08" y2="-5.08" width="0.4064" layer="94"/>
<text x="2.54" y="-7.62" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-10.16" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.032" y="-4.318" size="1.524" layer="95">GND</text>
<text x="-4.445" y="-0.635" size="1.524" layer="95">IN</text>
<text x="0.635" y="-0.635" size="1.524" layer="95">OUT</text>
<pin name="IN" x="-7.62" y="0" visible="off" length="short" direction="in"/>
<pin name="GND" x="0" y="-7.62" visible="off" length="short" direction="in" rot="R90"/>
<pin name="OUT" x="7.62" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="78XXL" prefix="IC" uservalue="yes">
<description>&lt;b&gt;VOLTAGE REGULATOR&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="78XX" x="0" y="0"/>
</gates>
<devices>
<device name="" package="78XXL">
<connects>
<connect gate="A" pin="GND" pad="GND"/>
<connect gate="A" pin="IN" pad="IN"/>
<connect gate="A" pin="OUT" pad="OUT"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-coax">
<description>&lt;b&gt;Coax Connectors&lt;/b&gt;&lt;p&gt;
Radiall  and M/A COM.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="A1944">
<description>&lt;b&gt;BNC CONNECTOR&lt;/b&gt;&lt;p&gt;
50 Ohm&lt;p&gt;
distributor Buerklin 78F2475</description>
<wire x1="-3.98" y1="-6" x2="3.98" y2="-6" width="0.254" layer="21"/>
<wire x1="6" y1="-3.98" x2="6" y2="3.98" width="0.254" layer="21"/>
<wire x1="3.98" y1="6" x2="-3.98" y2="6" width="0.254" layer="21"/>
<wire x1="-6" y1="3.98" x2="-6" y2="-3.98" width="0.254" layer="21"/>
<wire x1="-5.25" y1="6" x2="-6" y2="5.25" width="0.254" layer="51"/>
<wire x1="5.25" y1="6" x2="6" y2="5.25" width="0.254" layer="51"/>
<wire x1="6" y1="-5.25" x2="5.25" y2="-6" width="0.254" layer="51"/>
<wire x1="-5.25" y1="-6" x2="-6" y2="-5.25" width="0.254" layer="51"/>
<wire x1="-1.5" y1="0.5" x2="1.5" y2="0.5" width="0.3048" layer="21" curve="-143.130102"/>
<wire x1="-1.5" y1="-0.5" x2="1.5" y2="-0.5" width="0.3048" layer="21" curve="143.130102"/>
<wire x1="-4.572" y1="1.016" x2="-5.461" y2="1.016" width="0.254" layer="21"/>
<wire x1="-5.461" y1="1.016" x2="-5.461" y2="-1.016" width="0.254" layer="21"/>
<wire x1="-5.461" y1="-1.016" x2="-4.572" y2="-1.016" width="0.254" layer="21"/>
<wire x1="4.572" y1="-1.016" x2="5.461" y2="-1.016" width="0.254" layer="21"/>
<wire x1="5.461" y1="-1.016" x2="5.461" y2="1.016" width="0.254" layer="21"/>
<wire x1="5.461" y1="1.016" x2="4.572" y2="1.016" width="0.254" layer="21"/>
<wire x1="-6" y1="5.25" x2="-6" y2="3.81" width="0.254" layer="51"/>
<wire x1="-6" y1="-3.81" x2="-6" y2="-5.25" width="0.254" layer="51"/>
<wire x1="-5.25" y1="-6" x2="-3.81" y2="-6" width="0.254" layer="51"/>
<wire x1="6" y1="-5.25" x2="6" y2="-3.81" width="0.254" layer="51"/>
<wire x1="3.81" y1="-6" x2="5.25" y2="-6" width="0.254" layer="51"/>
<wire x1="-3.81" y1="6" x2="-5.25" y2="6" width="0.254" layer="51"/>
<wire x1="6" y1="3.81" x2="6" y2="5.25" width="0.254" layer="51"/>
<wire x1="5.25" y1="6" x2="3.81" y2="6" width="0.254" layer="51"/>
<circle x="0" y="0" radius="4.5961" width="0.254" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.778"/>
<pad name="2" x="5.08" y="5.08" drill="1.016" diameter="1.778"/>
<pad name="3" x="-5.08" y="5.08" drill="1.016" diameter="1.778"/>
<pad name="4" x="-5.08" y="-5.08" drill="1.016" diameter="1.778"/>
<pad name="5" x="5.08" y="-5.08" drill="1.016" diameter="1.778"/>
<text x="-2.54" y="6.35" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-7.62" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="BU-BNC">
<wire x1="-2.54" y1="2.54" x2="-2.54" y2="-2.54" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="0" y1="-2.54" x2="-0.762" y2="-1.778" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-0.508" y2="0" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="0.254" x2="-0.762" y2="0.254" width="0.254" layer="94"/>
<wire x1="-0.762" y1="0.254" x2="-0.508" y2="0" width="0.254" layer="94"/>
<wire x1="-0.508" y1="0" x2="-0.762" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-0.762" y1="-0.254" x2="-2.54" y2="-0.254" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="3.302" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<pin name="2" x="2.54" y="-2.54" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="A1944-50" prefix="X">
<description>&lt;b&gt;BNC CONNECTOR&lt;/b&gt; 50 Ohm&lt;p&gt;
distributor Buerklin 78F2475</description>
<gates>
<gate name="G$1" symbol="BU-BNC" x="0" y="0"/>
</gates>
<devices>
<device name="" package="A1944">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2 3 4 5"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply2">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
Please keep in mind, that these devices are necessary for the
automatic wiring of the supply signals.&lt;p&gt;
The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="SUPPLY">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="docu-dummy">
<description>Dummy symbols</description>
<packages>
</packages>
<symbols>
<symbol name="CAPACITOR">
<wire x1="0" y1="-5.08" x2="0" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="-0.508" x2="0" y2="2.54" width="0.1524" layer="94"/>
<rectangle x1="-2.032" y1="-2.032" x2="2.032" y2="-1.524" layer="94"/>
<rectangle x1="-2.032" y1="-1.016" x2="2.032" y2="-0.508" layer="94"/>
</symbol>
<symbol name="RESISTOR">
<wire x1="-2.54" y1="-0.889" x2="2.54" y2="-0.889" width="0.254" layer="94"/>
<wire x1="2.54" y1="0.889" x2="-2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="2.54" y1="-0.889" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.889" x2="-2.54" y2="0" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="-2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="-5.08" y1="0" x2="-2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="2.54" y1="0" x2="5.08" y2="0" width="0.1524" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C" prefix="C">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="CAPACITOR" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="R" prefix="R">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="RESISTOR" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="transistor">
<description>&lt;b&gt;Transistors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="TO92">
<description>&lt;b&gt;TO 92&lt;/b&gt;</description>
<wire x1="-2.0946" y1="-1.651" x2="-2.6549" y2="-0.254" width="0.127" layer="21" curve="-32.781"/>
<wire x1="-2.6549" y1="-0.254" x2="-0.7863" y2="2.5485" width="0.127" layer="21" curve="-78.3185"/>
<wire x1="0.7863" y1="2.5484" x2="2.0945" y2="-1.651" width="0.127" layer="21" curve="-111.1"/>
<wire x1="-2.0945" y1="-1.651" x2="2.0945" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.2537" y1="-0.254" x2="-0.2863" y2="-0.254" width="0.127" layer="51"/>
<wire x1="-2.6549" y1="-0.254" x2="-2.2537" y2="-0.254" width="0.127" layer="21"/>
<wire x1="-0.2863" y1="-0.254" x2="0.2863" y2="-0.254" width="0.127" layer="21"/>
<wire x1="2.2537" y1="-0.254" x2="2.6549" y2="-0.254" width="0.127" layer="21"/>
<wire x1="0.2863" y1="-0.254" x2="2.2537" y2="-0.254" width="0.127" layer="51"/>
<wire x1="-0.7863" y1="2.5485" x2="0.7863" y2="2.5485" width="0.127" layer="51" curve="-34.2936"/>
<pad name="1" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="3" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="3.175" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.175" y="-1.27" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-0.635" y="0.635" size="1.27" layer="51" ratio="10">2</text>
<text x="-2.159" y="0" size="1.27" layer="51" ratio="10">3</text>
<text x="1.143" y="0" size="1.27" layer="51" ratio="10">1</text>
</package>
</packages>
<symbols>
<symbol name="NPN">
<wire x1="2.54" y1="2.54" x2="0.508" y2="1.524" width="0.1524" layer="94"/>
<wire x1="1.778" y1="-1.524" x2="2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="1.27" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="1.778" y2="-1.524" width="0.1524" layer="94"/>
<wire x1="1.54" y1="-2.04" x2="0.308" y2="-1.424" width="0.1524" layer="94"/>
<wire x1="1.524" y1="-2.413" x2="2.286" y2="-2.413" width="0.254" layer="94"/>
<wire x1="2.286" y1="-2.413" x2="1.778" y2="-1.778" width="0.254" layer="94"/>
<wire x1="1.778" y1="-1.778" x2="1.524" y2="-2.286" width="0.254" layer="94"/>
<wire x1="1.524" y1="-2.286" x2="1.905" y2="-2.286" width="0.254" layer="94"/>
<wire x1="1.905" y1="-2.286" x2="1.778" y2="-2.032" width="0.254" layer="94"/>
<text x="-10.16" y="7.62" size="1.778" layer="95">&gt;NAME</text>
<text x="-10.16" y="5.08" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-2.54" x2="0.508" y2="2.54" layer="94"/>
<pin name="B" x="-2.54" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="E" x="2.54" y="-5.08" visible="off" length="short" direction="pas" swaplevel="3" rot="R90"/>
<pin name="C" x="2.54" y="5.08" visible="off" length="short" direction="pas" swaplevel="2" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="2N5088" prefix="T">
<description>&lt;b&gt;NPN TRANSISTOR&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="NPN" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TO92">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="1"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="IC1" library="silabs" deviceset="SI570" device=""/>
<part name="IC3" library="v-reg" deviceset="78XXL" device=""/>
<part name="X1" library="con-coax" deviceset="A1944-50" device=""/>
<part name="X2" library="con-coax" deviceset="A1944-50" device=""/>
<part name="SUPPLY1" library="supply2" deviceset="GND" device=""/>
<part name="C1" library="docu-dummy" deviceset="C" device=""/>
<part name="C2" library="docu-dummy" deviceset="C" device=""/>
<part name="SUPPLY2" library="supply2" deviceset="GND" device=""/>
<part name="R1" library="docu-dummy" deviceset="R" device=""/>
<part name="R2" library="docu-dummy" deviceset="R" device=""/>
<part name="C3" library="docu-dummy" deviceset="C" device=""/>
<part name="T1" library="transistor" deviceset="2N5088" device=""/>
<part name="SUPPLY3" library="supply2" deviceset="GND" device=""/>
<part name="R3" library="docu-dummy" deviceset="R" device=""/>
<part name="R4" library="docu-dummy" deviceset="R" device=""/>
<part name="R5" library="docu-dummy" deviceset="R" device=""/>
<part name="SUPPLY4" library="supply2" deviceset="GND" device=""/>
<part name="SUPPLY5" library="supply2" deviceset="GND" device=""/>
<part name="C4" library="docu-dummy" deviceset="C" device=""/>
<part name="C5" library="docu-dummy" deviceset="C" device=""/>
<part name="SUPPLY6" library="supply2" deviceset="GND" device=""/>
<part name="C6" library="docu-dummy" deviceset="C" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<frame x1="-215.9" y1="71.12" x2="-12.7" y2="210.82" columns="8" rows="5" layer="91"/>
<circle x="-203.2" y="198.12" radius="3.5921" width="0.1524" layer="91"/>
</plain>
<instances>
<instance part="IC1" gate="G$1" x="-185.42" y="152.4"/>
<instance part="IC3" gate="A" x="-152.4" y="180.34"/>
<instance part="X1" gate="G$1" x="-116.84" y="198.12"/>
<instance part="X2" gate="G$1" x="-30.48" y="147.32"/>
<instance part="SUPPLY1" gate="GND" x="-172.72" y="129.54"/>
<instance part="C1" gate="G$1" x="-160.02" y="175.26"/>
<instance part="C2" gate="G$1" x="-142.24" y="175.26"/>
<instance part="SUPPLY2" gate="GND" x="-152.4" y="165.1"/>
<instance part="R1" gate="G$1" x="-121.92" y="160.02" rot="R90"/>
<instance part="R2" gate="G$1" x="-121.92" y="149.86" rot="R90"/>
<instance part="C3" gate="G$1" x="-134.62" y="154.94" rot="R90"/>
<instance part="T1" gate="G$1" x="-109.22" y="154.94"/>
<instance part="SUPPLY3" gate="GND" x="-121.92" y="142.24"/>
<instance part="R3" gate="G$1" x="-106.68" y="144.78" rot="R90"/>
<instance part="R4" gate="G$1" x="-106.68" y="134.62" rot="R90"/>
<instance part="R5" gate="G$1" x="-106.68" y="165.1" rot="R90"/>
<instance part="SUPPLY4" gate="GND" x="-106.68" y="127"/>
<instance part="SUPPLY5" gate="GND" x="-96.52" y="127"/>
<instance part="C4" gate="G$1" x="-96.52" y="134.62"/>
<instance part="C5" gate="G$1" x="-93.98" y="172.72"/>
<instance part="SUPPLY6" gate="GND" x="-93.98" y="165.1"/>
<instance part="C6" gate="G$1" x="-96.52" y="157.48" rot="R90"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="GND"/>
<wire x1="-172.72" y1="144.78" x2="-172.72" y2="142.24" width="0.1524" layer="91"/>
<pinref part="SUPPLY1" gate="GND" pin="GND"/>
<pinref part="IC1" gate="G$1" pin="CLK-"/>
<wire x1="-172.72" y1="142.24" x2="-172.72" y2="132.08" width="0.1524" layer="91"/>
<wire x1="-172.72" y1="149.86" x2="-172.72" y2="144.78" width="0.1524" layer="91"/>
<junction x="-172.72" y="144.78"/>
</segment>
<segment>
<wire x1="-160.02" y1="170.18" x2="-152.4" y2="170.18" width="0.1524" layer="91"/>
<pinref part="IC3" gate="A" pin="GND"/>
<wire x1="-152.4" y1="170.18" x2="-142.24" y2="170.18" width="0.1524" layer="91"/>
<wire x1="-152.4" y1="172.72" x2="-152.4" y2="170.18" width="0.1524" layer="91"/>
<wire x1="-152.4" y1="170.18" x2="-152.4" y2="167.64" width="0.1524" layer="91"/>
<junction x="-152.4" y="170.18"/>
<pinref part="SUPPLY2" gate="GND" pin="GND"/>
<wire x1="-152.4" y1="167.64" x2="-152.4" y2="165.1" width="0.1524" layer="91"/>
<junction x="-152.4" y="167.64"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IC3" gate="A" pin="IN"/>
<wire x1="-160.02" y1="180.34" x2="-160.02" y2="177.8" width="0.1524" layer="91"/>
<wire x1="-160.02" y1="180.34" x2="-172.72" y2="180.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="IC3" gate="A" pin="OUT"/>
<wire x1="-144.78" y1="180.34" x2="-142.24" y2="180.34" width="0.1524" layer="91"/>
<wire x1="-142.24" y1="180.34" x2="-142.24" y2="177.8" width="0.1524" layer="91"/>
<wire x1="-142.24" y1="180.34" x2="-132.08" y2="180.34" width="0.1524" layer="91"/>
<wire x1="-132.08" y1="180.34" x2="-132.08" y2="160.02" width="0.1524" layer="91"/>
<pinref part="IC1" gate="G$1" pin="VDD"/>
<wire x1="-132.08" y1="160.02" x2="-172.72" y2="160.02" width="0.1524" layer="91"/>
<junction x="-142.24" y="180.34"/>
<wire x1="-121.92" y1="165.1" x2="-121.92" y2="177.8" width="0.1524" layer="91"/>
<wire x1="-121.92" y1="177.8" x2="-106.68" y2="177.8" width="0.1524" layer="91"/>
<wire x1="-106.68" y1="170.18" x2="-106.68" y2="177.8" width="0.1524" layer="91"/>
<wire x1="-106.68" y1="177.8" x2="-93.98" y2="177.8" width="0.1524" layer="91"/>
<wire x1="-93.98" y1="177.8" x2="-93.98" y2="175.26" width="0.1524" layer="91"/>
<junction x="-106.68" y="177.8"/>
<wire x1="-142.24" y1="180.34" x2="-106.68" y2="180.34" width="0.1524" layer="91"/>
<wire x1="-106.68" y1="180.34" x2="-106.68" y2="177.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="CLK+"/>
<wire x1="-172.72" y1="154.94" x2="-137.16" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="T1" gate="G$1" pin="B"/>
<wire x1="-129.54" y1="154.94" x2="-111.76" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<wire x1="-106.68" y1="139.7" x2="-96.52" y2="139.7" width="0.1524" layer="91"/>
<wire x1="-96.52" y1="139.7" x2="-96.52" y2="137.16" width="0.1524" layer="91"/>
<junction x="-106.68" y="139.7"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<wire x1="-106.68" y1="157.48" x2="-96.52" y2="157.48" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
