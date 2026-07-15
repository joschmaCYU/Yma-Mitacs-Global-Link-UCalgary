#include <SPI.h>

// ==========================================
// CONFIGURATION DES 3 BUS SPI MATÉRIELS
// L'ordre des pins pour STM32duino est : MOSI, MISO, SCK
// ==========================================
SPIClass SPI_1(PA7, PA6, PA1);
SPIClass SPI_2(PC3, PC2, PB10);
SPIClass SPI_3(PC12, PC11, PC10);

// Les paramètres SPI déduits de ton ancien code CubeMX :
// Polarity HIGH + Phase 1 Edge = SPI_MODE2
// Fréquence à 5 MHz (largement suffisant pour 32 bits)
SPISettings spiSettings(5000000, MSBFIRST, SPI_MODE2);

// Buffers pour stocker les 4 octets (32 bits) reçus
uint8_t buf1[4], buf2[4], buf3[4];

// ==========================================
// FONCTION DE DÉCODAGE DE LA TRAME SSI
// ==========================================
int16_t extract_position(uint8_t* buf) {
    unsigned char bitBuffer = 0; 
    uint16_t encoderposition = 0;
    int bitCount = 0; 
    bool foundStartSequence = false;

    // Parcours des 32 bits (4 octets * 8)
    for (int i = 0; i < 32; i++) {
        int byteIndex = i / 8;
        int bitIndex = 7 - (i % 8); 
        int currentBit = (buf[byteIndex] >> bitIndex) & 0x01;

        // Détection de la séquence "010"
        bitBuffer = ((bitBuffer << 1) | currentBit) & 0x07; 

        if (!foundStartSequence && bitBuffer == 0b010) {
            foundStartSequence = true;
            continue; 
        }

        // Extraction des 12 bits de position
        if (foundStartSequence) {
            if (bitCount < 12) { 
                encoderposition = (encoderposition << 1) | currentBit;
            }
            bitCount++;
            
            // On a ce qu'il nous faut, on peut quitter plus tôt
            if (bitCount >= 12) {
                return encoderposition;
            }
        }
    }
    return -1; // Code d'erreur si la séquence "010" n'est pas trouvée
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(2000); // Temps d'ouverture du port série

    // Initialisation matérielle des 3 bus SPI
    SPI_1.begin();
    SPI_2.begin();
    SPI_3.begin();

    Serial.println("--- LECTURE DES ENCODEURS SPI/SSI ---");
}

// ==========================================
// LOOP
// ==========================================
void loop() {
    // 1. Lecture de l'Encodeur 1
    SPI_1.beginTransaction(spiSettings);
    for(int i=0; i<4; i++) buf1[i] = SPI_1.transfer(0x00);
    SPI_1.endTransaction();

    // 2. Lecture de l'Encodeur 2
    SPI_2.beginTransaction(spiSettings);
    for(int i=0; i<4; i++) buf2[i] = SPI_2.transfer(0x00);
    SPI_2.endTransaction();

    // 3. Lecture de l'Encodeur 3
    SPI_3.beginTransaction(spiSettings);
    for(int i=0; i<4; i++) buf3[i] = SPI_3.transfer(0x00);
    SPI_3.endTransaction();

    // 4. Décodage des trames
    int16_t pos1 = extract_position(buf1);
    int16_t pos2 = extract_position(buf2);
    int16_t pos3 = extract_position(buf3);

    // 5. Affichage (à 10 Hz pour que ce soit lisible)
    Serial.print("MOT1: "); Serial.print(pos1);
    Serial.print("\t|\tMOT2: "); Serial.print(pos2);
    Serial.print("\t|\tMOT3: "); Serial.println(pos3);

    delay(100); 
}