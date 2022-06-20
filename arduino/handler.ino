#include <Encoder.h>
#include <AccelStepper.h>
#include <AS5X47.h>
#define ANGLE_REG         0x3FFE

// pin conf
#define pin_a_enc_cs 7
#define pin_a_stp 18
#define pin_a_dir 19
//#define pin_a_ena 17
#define pin_b_enc_cs 8
#define pin_b_stp 15
#define pin_b_dir 16
//#define pin_b_ena 14
#define pin_z_enc_a 3
#define pin_z_enc_b 2
#define pin_z_stp 4
#define pin_z_dir 5
#define pin_z_ena 6
#define pin_mag_en 9
#define pin_mag_pwm 10
#define mag_on digitalWrite(pin_mag_en, HIGH)
#define mag_off digitalWrite(pin_mag_en, LOW)
// rs bytes
#define b_start 0xFA
#define b_ignore 0xF0
byte b_jog   = 0;
#define b_jog_ap 0xEA
#define b_jog_am 0xEB
#define b_jog_bp 0xEC
#define b_jog_bm 0xED
#define b_jog_zp 0xEE
#define b_jog_zm 0xEF
byte b_aMotH = 0;
byte b_aMotL = 0;
byte b_aEncH = 0;
byte b_aEncL = 0;
byte b_bMotH = 0;
byte b_bMotL = 0;
byte b_bEncH = 0;
byte b_bEncL = 0;
byte b_zMotH = 0;
byte b_zMotL = 0;
byte b_zMag = 0;
byte b_calibrated = 0;
#define b_stop  0xFF
byte rs_buff[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte rs_byte_pos = 0;
byte rs_out_pos = 0;

AccelStepper a(AccelStepper::DRIVER, pin_a_stp, pin_a_dir);
AccelStepper b(AccelStepper::DRIVER, pin_b_stp, pin_b_dir);
AccelStepper z(AccelStepper::DRIVER, pin_z_stp, pin_z_dir); 
Encoder z_enc(pin_z_enc_a, pin_z_enc_b);
AS5X47 a_enc(pin_a_enc_cs);
AS5X47 b_enc(pin_b_enc_cs);

double z_lenght_cms = 51;
double z_lenght_steps = 0;
double z_mp_per_rev = 800;
double z_ep_per_rev = 2000;
double z_cm_per_rev = 8.08;
double z_motor_encoder_ratio = z_mp_per_rev / z_ep_per_rev;
double z_encoder_motor_ratio = z_ep_per_rev / z_mp_per_rev; 
double z_revs = 0;
int z_calibrated = 0;

double a_mp_per_rev = 3200;     // 1600 * 2 (beltdrive)
double a_ep_per_rev = 360;

double a_enc_average() {
    double average = 0;
    int x = 0;
    
    for ( int i = 0 ; i < 5 ; i++ ) {
        x = a_enc.readAngle();
        delay(10);
    }
    
    for ( int i = 0 ; i < 5 ; i++ ) {
        average += a_enc.readAngle();
        delay(100);
    }
    return average / 5;
}
double b_enc_average() {
    double average = 0;
    int x = 0;
    
    for ( int i = 0 ; i < 5 ; i++ ) {
        x = b_enc.readAngle();
        delay(10);
    }
    
    for ( int i = 0 ; i < 5 ; i++ ) {
        average += b_enc.readAngle();
        delay(100);
    }
    return average / 5;
}
long a_angle_to_steps(float angle) { return long(angle * a_mp_per_rev / a_ep_per_rev); }
int z_cms_to_steps(int cms) { return cms * z_mp_per_rev / z_cm_per_rev; }
int z_steps_to_cms(long steps) { return steps * z_cm_per_rev / z_mp_per_rev; }
int z_enc_err() { return abs( z_enc.read() * z_motor_encoder_ratio - z.currentPosition()); }
int z_calibrate() {
    long delta = 0;
    long max_delta = 0;     
    int z_error = 0;
    z.setMaxSpeed(1000);         z.setAcceleration(2000);

    
    z_enc.write(0);
    Serial.println("Program startup.");
    Serial.println("Going +20 steps (down)");  
    z.runToNewPosition(20);
    if ( z_enc_err() > 15 ) {
        //z_error = 1; 
        if ( z_enc_err() == 20 ) {
            Serial.println("Motor / Encoder disconnected on DOWN move. Aborting");
            
            return -1;
        }
        else
            Serial.println("Lost steps on DOWN move");
            Serial.print("Enc error after 20 steps: "); Serial.println(z_enc_err());
    }
    else {
        Serial.print("After +20 steps. Encoder error: "); Serial.println(z_enc_err());
        Serial.println(" ... OK");
        z_enc.write(0);
        z.setCurrentPosition(0);
    }
    delay(500);  

    Serial.println("Going -40 steps (up)");
    z.runToNewPosition(-40);

    if ( z_enc_err() > 15 ) {
        if ( z_error ) {
            Serial.println("Lost steps on both moves. Aborting");
            return -1;
        }
        else {
            Serial.println("Lost steps on UP move. Previously no errors on Down so this must be TOP limit");
            z_enc.write(0);
            z.setCurrentPosition(0);
            z.setMaxSpeed(700);
            z.setAcceleration(4000);
            z_calibrated = 1;
            return 1;
        }
    }
    else {
        Serial.print("After -40 steps. Encoder error: "); Serial.println(z_enc_err());
        Serial.println(" ... OK");
        z_enc.write(0);
        z.setCurrentPosition(0);
        Serial.println("Doing steps indefinetly UP");
        z.moveTo(-z_cms_to_steps(52));
            
        while ( z_enc_err() < 12 ) {
            z.run();
        }
        Serial.print("Moved: "); Serial.print(-z.currentPosition()); Serial.println(" steps."); 
        Serial.println("Calibrated."); 
        z_calibrated = 1;
        z_enc.write(0);
        z.setCurrentPosition(0);
        z.setMaxSpeed(700);
        z.setAcceleration(4000);
        return 1;
    }
}
int ab_calibrate() {
    double tempa = 0;
    double tempb = 0;
    Serial.println("A & B calibration");
    Serial.print("A encoder wake up position: "); Serial.print(a_enc.readAngle()); Serial.print("deg, converted to motor pos is: "); Serial.println(a_angle_to_steps(a_enc.readAngle()));
    Serial.print("B encoder wake up position: "); Serial.print(b_enc.readAngle()); Serial.print("deg, converted to motor pos is: "); Serial.println(a_angle_to_steps(b_enc.readAngle()));
    Serial.println("Setting current encoder position as motor position");
    a.setCurrentPosition(a_angle_to_steps(a_enc_average()));
    a.runToNewPosition(a_angle_to_steps(270));
    Serial.print("A going for: "); Serial.println(a_angle_to_steps(270));

    
    a.setCurrentPosition(a_angle_to_steps(a_enc_average()));
    a.runToNewPosition(a_angle_to_steps(270));

    Serial.println("Going for 180");
    b_precise(a_angle_to_steps(180));



    Serial.println("Going for 180");
    b_precise(a_angle_to_steps(180));

    Serial.print("A angle after calibration: "); Serial.println(a_enc.readAngle());     
    Serial.print("B angle after calibration: "); Serial.println(b_enc.readAngle());



}
void b_precise(long pos) {
    b.runToNewPosition(pos); 
    b.setCurrentPosition(a_angle_to_steps(b_enc.readAngle()));
    b.runToNewPosition(pos);
    b.setCurrentPosition(a_angle_to_steps(b_enc.readAngle()));
}
void send_stat() {
    long a_mot = a.currentPosition();
    b_aMotL = a_mot;
    b_aMotH = a_mot >>= 8;
    long a_en = a_enc.readAngleRaw();
    b_aEncL = a_en;
    b_aEncH = a_en >>= 8;
    long b_mot = b.currentPosition();
    b_bMotL = b_mot;
    b_bMotH = b_mot >>= 8;
    long b_en = b_enc.readAngleRaw();
    b_bEncL = b_en;
    b_bEncH = b_en >>= 8;
    long z_mot = z.currentPosition();
    b_zMotL = z_mot;
    b_zMotH = z_mot >>= 8;
    
    b_zMag = digitalRead(pin_mag_en);
    b_calibrated = 0;
    
    Serial.write(b_start);          // 0
    Serial.write(b_aMotH);          // 1
    Serial.write(b_aMotL);          // 2
    Serial.write(b_aEncH);          // 3
    Serial.write(b_aEncL);          // 4
    Serial.write(b_bMotH);          // 5
    Serial.write(b_bMotL);          // 6
    Serial.write(b_bEncH);          // 7
    Serial.write(b_bEncL);          // 8
    Serial.write(b_zMotH);           // 9
    Serial.write(b_zMotL);           // 10
    Serial.write(b_zMag);           // 11
    Serial.write(b_calibrated);     // 12
    Serial.write(b_stop);           // 13
    Serial.write(b_stop);

    Serial.println();

}
void receive_byte() {
    long a_target = 0;
    long b_target = 0;
    long z_target = 0;
    int z_mag = 0;
    
    
    //Serial.println("got a byte");
    switch ( rs_byte_pos ) {
        case 0:
            rs_buff[0] = Serial.read();
            if ( rs_buff[0] == b_start ) {        // pierwszy dobry pakiet w buforze. Krok naprzod
                //Serial.println("got 1");
                rs_byte_pos++; break;
            }
            break;
        case 1:
            rs_buff[1] = Serial.read();
            rs_byte_pos++; break;
        case 2:
            rs_buff[2] = Serial.read();
            rs_byte_pos++; break;
        case 3:
            rs_buff[3] = Serial.read(); 
            rs_byte_pos++; break;
        case 4:
            rs_buff[4] = Serial.read(); 
            rs_byte_pos++; break;
        case 5:
            rs_buff[5] = Serial.read(); 
            rs_byte_pos++; break;
        case 6:
            rs_buff[6] = Serial.read(); 
            rs_byte_pos++; break;
        case 7:
            rs_buff[7] = Serial.read(); 
            rs_byte_pos++; break;
        case 8:
            rs_buff[8] = Serial.read(); 
            rs_byte_pos++; break;
        case 9:
            rs_buff[9] = Serial.read(); 
            if ( rs_buff[9] == b_stop ) {
                if ( rs_buff[1] != b_ignore ) {
                  if ( rs_buff[1] == b_jog_ap ) 
                    a.runToNewPosition(a.currentPosition()+10); 
                  if ( rs_buff[1] == b_jog_am ) 
                    a.runToNewPosition(a.currentPosition()-10); 
                  if ( rs_buff[1] == b_jog_bp ) 
                    b.runToNewPosition(b.currentPosition()+10); 
                  if ( rs_buff[1] == b_jog_bm ) 
                    b.runToNewPosition(b.currentPosition()-10); 
                  if ( rs_buff[1] == b_jog_zp ) 
                    z.runToNewPosition(z.currentPosition()+5); 
                  if ( rs_buff[1] == b_jog_zm && z.currentPosition() > 5 )
                    z.runToNewPosition(z.currentPosition()-5); 
                }

                
                if ( rs_buff[2] != b_ignore ) {
                    a_target = rs_buff[2];
                    a_target <<= 8;
                    a_target |= rs_buff[3];
                    Serial.println(a_target);
                    a.runToNewPosition(a_target);
                }
                
                if ( rs_buff[4] != b_ignore ) {
                    b_target = rs_buff[4];
                    b_target <<= 8;
                    b_target |= rs_buff[5];
                    b_precise(b_target);
                    
                }
                
                if ( rs_buff[6] != b_ignore ) {
                    z_target = rs_buff[6];
                    z_target <<= 8;
                    z_target |= rs_buff[7];
                    if ( z_target > -1 && z_steps_to_cms(z_target) < z_lenght_cms ) {
                        z.runToNewPosition((z_target));
                    }
                }

                if ( rs_buff[8] != b_ignore ) {
                    if ( rs_buff[8] )
                      mag_on;
                    else
                      mag_off;
                }
            }
            rs_byte_pos = 0; break;
        default:
            rs_byte_pos = 0; break;
    }
}
void pickup() {
      z.runToNewPosition(190);
      mag_on;
      z.runToNewPosition(0);
      a.runToNewPosition(a_angle_to_steps(90));
      z.runToNewPosition(z_cms_to_steps(51));
      delay(5000);
      z.runToNewPosition(0);
      a.runToNewPosition(a_angle_to_steps(270));
      z.runToNewPosition(190);
      mag_off;
      z.runToNewPosition(0);
      
}
void debug_encoder() {
    
    
    //Serial.println("Dropping the motor");
    //digitalWrite(pin_z_ena, LOW);
    while(1) {
        //update_pos();
        //Serial.print("Enc RAW = "); Serial.print(z_enc.read()); Serial.print(" | Enc (cm) = "); Serial.print(z_steps_to_cms(z_enc.read())); Serial.print(" | Enc (revs) = "); Serial.println(double(z_enc.read()/z_ep_per_rev));
        Serial.print("A: "); Serial.print(a_enc.readAngle()); Serial.print(" B:"); Serial.print(b_enc.readAngle()); Serial.print(" Z:"); Serial.println(z_enc.read());
    }
}

void setup() {
    z_lenght_steps = z_cms_to_steps(z_lenght_cms);
    
    z.setPinsInverted(true, false, false);
    a.setPinsInverted(true, false, false);
    b.setPinsInverted(true, false, false);
    z.setMaxSpeed(1600);         z.setAcceleration(2000);
    a.setMaxSpeed(800);         a.setAcceleration(1000);
    b.setMaxSpeed(3000);         b.setAcceleration(600);    
    
    
    
    Serial.begin(115200, SERIAL_8E2);
    pinMode(pin_z_stp, OUTPUT);
    pinMode(pin_z_ena, OUTPUT);
    //pinMode(pin_a_ena, OUTPUT);
    //pinMode(pin_b_ena, OUTPUT);


    
    pinMode(pin_z_dir, OUTPUT);
    digitalWrite(pin_z_ena, HIGH);

    //digitalWrite(pin_a_ena, HIGH);
    //digitalWrite(pin_b_ena, HIGH);

    pinMode(pin_mag_en, OUTPUT);
    pinMode(pin_mag_pwm, OUTPUT);
    mag_off;
    analogWrite(pin_mag_pwm, 200);
    

    
//    z.setMaxSpeed(1600);
//    z.setAcceleration(1000);
//    z.runToNewPosition(-800);
//    delay(1000);
//    z.runToNewPosition(-3200);
    z_calibrate();
    
        ab_calibrate();
        //check_collision_on_down_move();
        //pickup();
        //a.runToNewPosition(-3200);
    //debug_encoder();
    //while(1);
    
    //linearity_check();
}
void loop() {
    send_stat();
    //Serial.print("A_pos_drv_raw: "); Serial.println(a.currentPosition());
    if ( Serial.available() > 0 ) receive_byte();
    delay(10);



}
