//  Sabertooth 2*32 radio mode
#ifndef SABERTOOTH232_H
#define SABERTOOTH232_H

void writeMotor (float duty_right, float duty_left){
    if (OSW2&1){
        MD_R.pulsewidth_us(1500);
        MD_L.pulsewidth_us(1500);
    }else{
        if (duty_right > 1.0f)
            duty_right = 1.0f;
        else if (duty_right < -1.0f)
            duty_right = -1.0f;
            
        if (duty_left > 1.0f)
            duty_left = 1.0f;
        else if (duty_left < -1.0f)
            duty_left = -1.0f;
        
        MD_R.pulsewidth_us(1500+(int)(duty_right*500));
        MD_L.pulsewidth_us(1500+(int)(duty_left*500));
    }

}

#endif //SABERTOOTH232_H
