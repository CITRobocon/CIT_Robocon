#include "mbed.h"
#include "PS_PAD.h"

PS_PAD vsc3(D11,D12,D13,D10);

int main() {
    vsc3.init();
    printf("Hello World\r\n");
    
    while(1) {
        vsc3.poll();
        if (vsc3.read(PS_PAD::PAD_CIRCLE))
            printf("a\r\n");
        else if (vsc3.read(PS_PAD::PAD_SQUARE))
            printf("b\r\n");
        else if (vsc3.read(PS_PAD::PAD_TRIANGLE))
            printf("c\r\n");
        else if (vsc3.read(PS_PAD::PAD_X))
            printf("d\r\n");
    }
}
