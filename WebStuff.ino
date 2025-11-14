
void handleCmd() 
{
  String out;
  float test;

  for (uint8_t i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "sip") RemoteIP.fromString(server.arg(i));
  }
  
  out += "<html><br><br><center>\n";
    
  out += "Local IP ";
  out += act_ip;
  out += "<br>port ";
  out += localUdpPort;
  out += "<br>MAC ";
  out += WiFi.macAddress();
  out += "<br><br>";

  out += "<form method=\"post\">\n";
  out += "Remote IP &emsp;\n";
  out += "<input type=\"text\" name=\"sip\" value=\"";
  out += RemoteIP.toString().c_str();
  out += "\"/>\n";
  out += "<input type=\"submit\"><br>\n";
  out += "</form>\n";

  out += "udp rx  ";  out += UDPrxnum;  out += "<br>";
  out += "udp tx  ";  out += UDPtxnum;  out += "<br>";
  out += "lora rx ";  out += LORArxnum; out += "<br>";
  out += "lora tx ";  out += LORAtxnum; out += "<br>";
  out += "<br>";
  
  out += "<a href=\"/\">Refresh</a>\n";
  out += "</center></html>";
  server.send(200, "text/html", out);    
}

void handleNotFound() 
{
  server.send(404, "text/plain", "Not Here");
}
