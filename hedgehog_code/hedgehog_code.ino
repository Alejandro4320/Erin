int hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), cm
int hedgehog_h;// height of hedgehog, cm
int hedgehog_pos_updated;// flag of new data from hedgehog received

///

#define HEDGEHOG_BUF_SIZE 30 
#define HEDGEHOG_PACKET_SIZE 23
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

const int hedgehog_packet_header[5]= {0xff, 0x47, 0x01, 0x00, 0x10};

typedef union {byte b[2]; unsigned int w;} uni_8x2_16;

//    Marvelmind hedgehog support initialize
void setup_hedgehog() 
{
  Serial.begin(115200); // hedgehog transmits data on 500 kbps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;
}

// Marvelmind hedgehog service loop
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;

  total_received_in_loop= 0;
  packet_received= 0;
  while(Serial.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) break;// buffer overflow
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= Serial.read();
      // check first 5 bytes for constant value
      if (hedgehog_serial_buf_ofs<5)
        {
          if (incoming_byte != hedgehog_packet_header[hedgehog_serial_buf_ofs]) 
            {
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              continue;
            }
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte;  
      if (hedgehog_serial_buf_ofs == HEDGEHOG_PACKET_SIZE)
        {// received packet with required header
          packet_received= 1;
          hedgehog_serial_buf_ofs= 0;// restart bufer fill
          break; 
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], HEDGEHOG_PACKET_SIZE);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[HEDGEHOG_PACKET_SIZE] == 0)&&(hedgehog_serial_buf[HEDGEHOG_PACKET_SIZE+1] == 0))
        {// checksum success
          hedgehog_x= int(hedgehog_serial_buf[9]) + (int(hedgehog_serial_buf[10])<<8);
          hedgehog_y= int(hedgehog_serial_buf[11]) + (int(hedgehog_serial_buf[12])<<8);// coordinates of hedgehog (X,Y), cm
          hedgehog_h= int(hedgehog_serial_buf[13]) + (int(hedgehog_serial_buf[14])<<8);// height of hedgehog, cm (FW V3.97+)
          hedgehog_pos_updated= 1;// flag of new data from hedgehog received
        }
    }
}

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// hedgehog_set_crc16

//  END OF MARVELMIND HEDGEHOG RELATED PART
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
     //Echo_pin


void setup()
{ 
  Serial.begin(115200);
  setup_hedgehog();//    Marvelmind hedgehog support initialize
}

void loop()
{ 
  loop_hedgehog();// Marvelmind hedgehog service loop

   if (hedgehog_pos_updated)
     {// new data from hedgehog available
       hedgehog_pos_updated= 0;// clear new data flag 
       // output hedgehog position to LCD
Serial.print("X=");
Serial.println(hedgehog_x);
Serial.print("   ");  
       
Serial.print("Y=");
Serial.println(hedgehog_y);
Serial.print("   ");  

Serial.print("H=");
Serial.println(hedgehog_h);
Serial.print("   ");  
     }
}
