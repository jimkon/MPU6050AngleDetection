
## Menu
1. [Sensor](#sensor)
  1. [Details](#details)
  2. [Configuration](#configuration)

## Sensor
 
Ο αισθητήρας που χρησιμοποιήθηκε είναι ο MPU6050 (GY 521 breakout board).

## Details

   # Sampling rate
   
   Μπορούμε να ελέγξουμε τον ρυθμό με τον οποίο δειγματοληπτούνται οι αισθητήρες για καθορίσουμε τον χρόνο με τον οποίο θα πρέπει να διαβάζουμε δεδομένα από την FIFO πριν γεμίσει.  
   Ο ρυθμός δειγματοληψίας καθορίζεται από την σχέση:  
      Sample_rate = (Gyroscope sample rate)/(1+SMPLRT_DIV[7:0])  
   Gyroscope sample rate: 8kHz για 0<DLPF_CFG[2:0]<7
                          1kHz αλλιώς  
   Όπως είναι προφανές, ο ρυθμός δειγματοληψίας κυμαίνεται από 8kHz μέχρι ~= 4 Hz. 
   

## Configuration
