/*
arduino-cli compile  --fqbn esp32:esp32:esp32                                                                          
arduino-cli compile  --fqbn esp32:esp32:esp32:PartitionScheme=default_8MB  . 
arduino-cli compile  --fqbn esp32:esp32:esp32:PartitionScheme=default_16MB  .

arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32:PartitionScheme=default_16MB,FlashMode=qout .

arduino-cli upload -p /dev/ttyUSB0 --fqbn  esp32:esp32:esp32 .        
arduino-cli upload -p /dev/ttyUSB0 --fqbn  esp32:esp32:esp32:FlashMode=dout .


esptool  flash_id
esptool  --port /dev/ttyUSB1 flash_id 

 
scp  build/esp32.esp32.esp32/qc-device.ino.bin  frenell@frenell-MotorTeststation.local:.
ssh frenell@frenell-MotorTeststation.local

esptool.py    --port /dev/ttyUSB1 --baud 921600 write_flash 0x00010000 qc-device.ino.bin 
*/

#include <fNimBLE.h>
#include <fJSON.h> 
#include <esp32can.h>
#include <SerialCin.h> 
#include <iostream>
#include <string>

using namespace std;

Esp32Can can(Serial);
int baud=50;
SerialCin myserial(Serial);   

fNimBLE *fble;                                                                                                          
                                                                                                                        
using namespace fjson;  

char compiledate[] = {__DATE__ " "__TIME__};   
char progversion[]= {"V0.0"};
JObj jinfo;



//0x700 : CAN msg ID
//1: number of data bytes in the msg
//0x5 the value of the single data byte in the msg 
CanMsg chb = {0x700+1, 1, {0x5}};   



int nodeid=101; // standard pcb
struct motordata {
int pos;
int speed;
};

motordata mdl,mdr;
JObj jdata;
CanNode cannode(nodeid); 
CanNode cannodeml(10); //can-node for motor left
CanNode cannodemr(11); //can-node for motor right   

unsigned int lctr=0;
bool b_sendhb=true;
bool b_sendsync=true;
bool b_sendcanviabt=true;

void can_cb(CanMsg &msg){
//Serial.print("can_cb 0x");
//Serial.print(msg.id,HEX);
//Serial.println("  ");
char cstr[32];
canmsg2str(msg,cstr);

if (b_sendcanviabt) sendbt(cstr);
}

//-----------------------------------------------------------------------------
void canmsg2str(CanMsg &msg,char *cstr32){
if (msg.id>4095) {
	          sprintf(cstr32,"cane%08x",msg.id);
	          
		 }
else {
snprintf(cstr32,32,"can%03x",msg.id);
}

for (int i=0;i<msg.len;i++) {
	 char cstr2[4];
	 snprintf(cstr2,4,"%02x",msg.data[i]);
	 strcat(cstr32,cstr2);
	                    }
}

//-----------------------------------------------------------------------------
void cannode_cb(CanMsg &msg){
 int nodeid=0x7F & msg.id;           
 int pdo=msg.id-nodeid;
 cout << "cannode_cb "<<nodeid << " 0x"<<hex << msg.id << " 0x" << pdo<< "  "  << dec << endl;
      
}
//-----------------------------------------------------------------------------
bool canmsgtxt2msg(const string m,CanMsg &canmsg){
int l=m.length();  
if (l<3+3+1*2) return false;
if (m[0]=='c' && m[1]=='a' && m[2]=='n'){
 int len=(l-(3+3)); 
 if (len%2!=0) return false;
 len/=2;

 char cstr[3]={m[3],m[4],m[5]};
 try {
 canmsg.id=std::strtoul(cstr, nullptr, 16);
 cstr[2]=0x0;
 char data[8];
 for (int i=0;i<len;i++){
 cstr[0]=m[6+i*2];
 cstr[1]=m[6+i*2+1]; 
 canmsg.data[i]=std::strtoul(cstr, nullptr, 16);
 }
 canmsg.len=len;
 cout << " canmsg "<< len  << " " << hex << "0x"<< canmsg.id << " ";
 for (int i=0;i<len;i++){
	 cout << " 0x" << int(canmsg.data[i] & 0xff); 
	              }

 cout << dec << endl;
 return true;
 }
 catch (...){
 cout << "wrong format " << endl;
 return false;
  }
 }
return false; 
}
//-----------------------------------------------------------------------------
void sendbt(const string s){
 if (fble->isconnected()) {
  fble->updatecharacteristic("2001",s.c_str(),true);
 }
}
//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("starting..");

  can.setcallback(&can_cb);
  can.setNodecallback(nodeid,&cannode_cb);


  // get motorposition and speed via pdo
  function<void(int,CanMsg&)> mpdofunc=[](int nrpdo,CanMsg &msg){
  int nodeid=0x7F & msg.id;
  int pdo=msg.id-nodeid;
  cout << "cannode_cb "<<nodeid << " 0x"<<hex << pdo << dec << " nr " << nrpdo << endl; 
            int ctr=msg.data[0];
	    ctr+=msg.data[1]<<8;
	    ctr+=msg.data[2]<<16;
	    ctr+=msg.data[3]<<24;
	    int speed=msg.data[4];
	        speed+=msg.data[5]<<8; 
	        speed+=msg.data[6]<<16;
		speed+=msg.data[7]<<24;
                speed/=10.;

 cout << "cannode_cb "<<nodeid << " 0x"<<hex << pdo << dec << " nr " << nrpdo << " pos " 
	 << ctr << " speed " << speed << endl; 


   if (nodeid==10 && pdo==0x180) {
	       mdl.pos=ctr;
	       mdl.speed=speed;
	       return;
	          }

   if (nodeid==11 && pdo==0x180) {
	       mdr.pos=ctr;
	       mdr.speed=speed;
	       return;
	          }
  };
  cannodeml.onpdo(mpdofunc); 
  cannodemr.onpdo(mpdofunc);


  cannode.onpdo([](int nrpdo,CanMsg &msg) {
    cout << "AD " << nrpdo << " " << endl;
	if (nrpdo==1) { // txpdo1 0 txpdo2 1
        int ma=msg.data[0];
	ma+=msg.data[1]<<8;
        jdata["a"][4]=ma/1000.;
	 ma=msg.data[2];
	 ma+=msg.data[3]<<8;
	 jdata["a"][5]=ma/1000.; 
                 }
   });
  //-----------------------------------------------------------------------------
  fble=new fNimBLE("qc-device","7f190f68-2749-493c-947f-193fe27fd4cb");      
  fble->addcharacteristic("2001",NIMBLE_PROPERTY::READ |  NIMBLE_PROPERTY::NOTIFY);      
  fble->addcharacteristic("2002",NIMBLE_PROPERTY::READ |  NIMBLE_PROPERTY::NOTIFY); 
  fble->addcharacteristic("2010",NIMBLE_PROPERTY::WRITE);  
  fble->addcallbck("2010"); // to (WRITABLE cahracteristic )
  fble->ongetline([](string m){
		         m.pop_back(); // erase ^M
		         cout << "fble ongetline " << m.length() << " " << m << endl; // ^M 0x0d  ?   

		  if (m.find("can")==0) {
                                      //cout << m.substr(3) << endl;
		                      CanMsg msg;
                                      if (canmsgtxt2msg(m,msg)) can.send(msg);
		                      return;
		                      }

		  if (m.find("{")!=0) {
		                      cout << "no json " << endl;
		                      return; 
				      }
                 JObj jo;
                 jo.create(m);
                 cout <<"json: "<< jo.str() << endl;
		 
		 if (!jo["act"].isVoid()) {
		    if (jo["act"].getString()=="info") {
			     sendbt(jinfo.str());
			                               }

			 return;
		                           }
		 
		 if (!jo["can"].isVoid()) {
		                cout << "-> " << jo["can"].getString() << endl;
		                if (jo["can"].getString()=="stop") {
				                b_sendcanviabt=false;
				                       }
		                if (jo["can"].getString()=="start") {
				                b_sendcanviabt=true;
				                       }
		 return;                         
		 }

		 if (!jo["do"].isVoid()) {  
			  if (!jo["do"]["ml"].isVoid()) {
				         int speed=jo["do"]["ml"]["s"].getInt();
					 cannodeml.sendSDO(0x22,0x3300,0x0,speed);
					 delay(2);
		                         int enable=0;
					 if (speed!=0) enable=1;
					 cannodeml.sendSDO(0x2f,0x3004,0x0,enable);
                                         cout << "sendsdo ml " << speed <<  endl;
					 return;
				                        }

			  if (!jo["do"]["mr"].isVoid()) {
				         int speed=jo["do"]["mr"]["s"].getInt();
					 cannodemr.sendSDO(0x22,0x3300,0x0,speed);
					 delay(2);
					 int enable=0;
					 if (speed!=0) enable=1;
					 cannodemr.sendSDO(0x2f,0x3004,0x0,enable);
					 cout << "sendsdo mr " << speed <<endl;
					 return;
				                        }
                                       }

		               });
   fble->start();    
   //-----------------------------------------------------------------------------



  if (ESP.getChipRevision()>=3) baud=100;
  can.start(baud);  // 50 for E version
  cout << "can baud " << baud << "  chiprevisionchiprevision  " << int(ESP.getChipRevision()) << endl;
  can.setprint(false);



  myserial.onmsg([](const char *in){
		  cout << "in: " << in << endl;
                  char key[64];
                  sscanf(in,"%s",key);
                  cout << "key " << key << endl;
		  if (!strcmp(key,"reboot")) {
		                              ESP.restart();
					      }

     if (!strcmp(key, "sendascii")) {
	     // 11bit cobid
	     // sendascii can0804004300100010203   payload 22 
	     // 29bit cobid
	     // 0x3fffffff    8 bytes
	     // endascii cane3fffffff4004300100010203  ? 
	     char txt[64];
	     sscanf(in,"sendascii %64s",txt);
             sendbt(txt);
	     return;
	                            }


		  if (!strcmp(key,"canprint")) { can.setprint(true);return;}
		   if (!strcmp(key,"cannoprint")) {can.setprint(false);return;}
		   if (!strcmp(key,"sendsdo"))  {
		   int node=12;
		   sscanf(in,"sendsdo %d",&node);
                   if (node>127) node=127;
		   if (node<0) node=0;
		   can.sendSDO(node,0x40,0x1018,0x1,0);return;
		   
		   }
		   if (!strcmp(key,"sendhb")) {b_sendhb=true;return;}
		    if (!strcmp(key,"sendsync"))  {b_sendsync=true;return;} 
                    if (!strcmp(key,"sendnosync")) {b_sendsync=false;return;}
                   if (!strcmp(key,"sethb")) {
		       int nodeid=0;
		       int ret=sscanf(in,"sethb %d",&nodeid);
                        cout << "ret " << ret << " " << nodeid << endl;
		       if (nodeid>127 || nodeid<0) return;
                       cout << "set heartbeat for " << nodeid << endl;
		       can.sendSDO(nodeid,0x2b,0x1017,0x0,1000);
		       return;
		                            }

                   if (!strcmp(key,"boot")) {
		       int nodeid=0;
		       int ret=sscanf(in,"boot %d",&nodeid);
                        cout << "ret " << ret << " " << nodeid << endl;
		       if (nodeid>127 || nodeid<0) return;
		       CanMsg m = {0x0, 2, {0x81,nodeid}}; 
		       can.send(m);
		       return;
		                            }

                   if (!strcmp(key,"setop")) {
		       int nodeid=0;
		       int ret=sscanf(in,"setop %d",&nodeid);
                        cout << "ret " << ret << " " << nodeid << endl;
		       if (nodeid>127 || nodeid<0) return;
		       CanMsg m = {0x0, 2, {0x1,nodeid}}; 
		       can.send(m);
		       return;
		                            }
		      if (!strcmp(key,"canreset")) { 
                     CAN.canreset();
		     return;
		      }

                   if (!strcmp(key,"setpreop")) {
		       int nodeid=0;
		       int ret=sscanf(in,"setpreop %d",&nodeid);
                        cout << "ret " << ret << " " << nodeid << endl;
		       if (nodeid>127 || nodeid<0) return;
		       CanMsg m = {0x0, 2, {0x80,nodeid}}; 
		       can.send(m);
		       return;
		                            }
		    if (!strcmp(key,"sync")) {
			    can.sync(0x80);
			    return;
			                     }
                     if (!strcmp(key,"canmsg")) { // canmsg can08040181001
			                         // command 11 0x580+nodeid
			                         // command 12 0x600+nodeid
			                          // node120 command 11 0x5f8    canmsg can5f840181001aabbccdd


                             char txt[64];
			     sscanf(in,"canmsg %64s",txt);
			     CanMsg canmsg;
			     if (canmsgtxt2msg(txt,canmsg)) {
				                          cout << "send to " << (0x7f & canmsg.id) << endl; 
				                          can.send(canmsg);
							   }
			     return;
		     }


		   if (!strcmp(key,"nosendhb")) b_sendhb=false;
		   if (!strcmp(key,"help")) {
		                 cout << "help for ..." << endl;
				 cout << "canprint" << endl;
				 cout << "cannoprint" << endl;
                                 cout << "sendsdo " << endl;
				 cout << "sendhb " << endl;
				 cout << "nosendhb " << endl;
				 cout << "sethb <nodeid>" << endl;
				 cout << "boot <nodeid> " << endl;
				 cout << "sendsync" << endl;
                                 cout << "sendnosync" << endl;
				 cout << "sync" << endl; 
				 return;
		                           }
		   cout << key << "  not found " << endl;
                  					   
		  });

  jinfo.addString("version",progversion).addString("compiled",compiledate);

  jdata.addObj("ml").addInt("p",0).addDouble("s",0.0);
  jdata.addObj("mr").addInt("p",0).addDouble("s",0.0);
  jdata.addArray("a").addDouble(0.0).addDouble(0.0).addDouble(0.0).addDouble(0.0).addDouble(0.0).addDouble(0.0);

  cout << "jdata " << jdata.str() << endl;
}

//-----------------------------------------------------------------------------
void loop() {
lctr++;	
myserial.check();
if (lctr%10==0) {
cout << "* " << lctr << " " << endl;
can.send(chb);
}
if (b_sendsync && lctr%11==0) can.sync();

if (lctr%10==0) {
    jdata["ml"]["p"]=mdl.pos;
     jdata["ml"]["s"]=mdl.speed;
    jdata["mr"]["p"]=mdr.pos; 
    jdata["mr"]["s"]=mdr.speed;  
  
    
    sendbt(jdata.str());
    cout << "jdata " << jdata.str();
	        }

delay(100);
}
