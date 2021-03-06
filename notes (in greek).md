
# Menu
1. [Sensor](#sensor)
  1. [Details](#details)
  2. [Configuration](#configuration)

# Sensor
 
Ο αισθητήρας που χρησιμοποιήθηκε είναι ο MPU6050 (GY 521 breakout board).

## Details

1. ### Sensitivity
    
      Το εύρος των τιμών που δίνουν οι αισθητήρες είναι από -32768 έως 32767, δηλαδή 65536 τιμές συνολικά. Ανάλογα με την ευαισθησία που θα επιλέξουμε για το επιταχυνσιόμετρο και το γυροσκόπιο, καθορίζουμε το εύρος μετρήσεων στο οποίο αντιστοιχούν οι τιμές αυτές. Έτσι προκύπτει ο παρακάτω πίνακας:
   
   Sensor   |   Sel   |   Min   |   Max   |   Total   |   LSB Sensitivity   |   Sensitivity
   ---|---|---|---|---|---|---
   Accelerometer|0|-2 g|+2 g|4 g|16384 LSB/g|0.06103515625 mg/LSB
   -|1|-4 g|+4 g|8 g|8192 LSB/g|0.1220703125 mg/LSB
   -|2|-8 g|+8 g|16 g|4096 LSB/g|0.244140625 mg/LSB
   -|3|-16 g|+16 g|32 g|2048 SB/g|0.48828125 mg/LSB
   Gyroscope|0|-250 °/s|+250 °/s|500 °/s|131 LSB/°/s|0.00763358778625954198473 °/s/LSB
   -|1|-500 °/s|+500 °/s|1000 °/s|65.5 LSB/°/s|0.01526717557251908396946 °/s/LSB
   -|2|-1000 °/s|+1000 °/s|2000 °/s|32.8 LSB/°/s|0.03053435114503816793893 °/s/LSB
   -|3|-2000 °/s|+2000 °/s|4000 °/s|16.4 LSB/°/s|0.06106870229007633587786 °/s/LSB
   
      Από τον παραπάνω πίνακα μπορούμε να διακρύνουμε και να επιλέξουμε την μέγιστη ευαισθησία ανάλογα την "ταχύτητα" που θέλουμε να μπορεί να ανταποκριθεί η εφαρμογή μας. 
   
   * __setRangeSettings(int accel, int gyro):__ Θέτει τις επιλογές ευαισθησίας στα αντίστοιχα πεδία, 0<=accel<=3 , 0<=gyro<=3
   * __getAngleX():__ Επιστρέφει την κλίση του chip ώς προς τον άξονα Χ
   * __getAngleΥ():__ Επιστρέφει την κλίση του chip ώς προς τον άξονα Υ
   * __getAngleΖ():__ Επιστρέφει την κλίση του chip ώς προς τον άξονα Ζ

2. ### Sampling rate
   
      Μπορούμε να ελέγξουμε τον ρυθμό με τον οποίο δειγματοληπτούνται οι αισθητήρες για καθορίσουμε τον χρόνο με τον οποίο θα πρέπει να διαβάζουμε δεδομένα από την FIFO πριν γεμίσει.     
   
      Ο ρυθμός δειγματοληψίας καθορίζεται από την σχέση:  
         __Sample_rate = (Gyroscope sample rate)/(1+SMPLRT_DIV[7:0])__
         
   όπου Gyroscope sample rate:
      * 8kHz για 0<DLPF_CFG[2:0]<7  
      * 1kHz αλλιώς    
   
     Όπως είναι προφανές, ο ρυθμός δειγματοληψίας κυμαίνεται από 8kHz μέχρι ~= 4 Hz. Να αναφερθεί ότι ο ρυθμός δειγματοληψίας για το επιταχυνσιόμετρο δεν μπορεί να υπερβεί το 1kHz. Για ρυθμό μεγαλύτερο από 1kHz, τα δεδομένα που θα παρέχονται θα είναι αντίγραφα προηγούμενων τιμών.

   * __setSampleRate(int rate):__    Θέτει ρυθμό δειγματοληψίας τον κοντινότερο ακέραιο στο rate
   * __getSampleRate():__   Επιστρέφει τον ρυθμό δειγματοληψίας

3. ### FIFO
   
      Ο κατασκευαστής αναφέρει ότι αν δεν χρησιμοποιηθεί η στοίβα, τότε ο χρήστης πρέπει να διασφαλίσει ελέγχοντας το Data Ready interrut ότι δεν θα διαβάσει δεδομένα την στιγμή που νέα δεδομένα γράφονται στον ίδιο καταχωρητή. Έτσι για λόγους εξοικονόμησης ενέργειας, ακρίβειας, και απλότητας, χρησιμοποίηθηκε η στοίβα όπου μαζέυει δεδομένα ώστε να διαβαστούν μία και καλή. 
      
      Το μέγεθος της στοίβας ειναι 1024 byte, έτσι υπάρχει ένας συγκεκριμένος χρόνος στον οποίο γεμίζει, ο οποίος εξαρτάται από το [ρυθμό δειγματοληψίας](#sampling-rate) αλλά και από τους ενεργοποιημένους άξονες του αισθητήρα. Ο χρήστης μπορεί να επιλέξει ποιοί άξονες περιστροφής είναι ενεργοποιημένοι, έτσι ώστε να απενεργοποιήσει άξονες που δεν χρειάζεται και να μεγαλώσει τον παραπάνω χρόνο. Σε περίπτωση που γεμίσει η στοίβα, τα νέα δεδομένα θα πανωγράφουν τα παλαιότερα εισαχθέντα σε αυτήν. Σε κάθε περίοδο γράφονται στην στοίβα 6 byte που αντιστοιχούν στους 3 άξονες ΧΥΖ του επιταχυνσιόμετρου που δεν μας αφήνει ο κατασκευαστής να τους διαχωρίσουμε, και 2 byte για κάθε ενεργοποιημένο άξονα του γυροσκόποιου. Για παράδειγμα, αν ενεργοποιήσουμε μόνο τους άξονες Χ και Ζ, σε κάθε περίοδο θα γράφονται στην στοίβα ΧΥΖ_accel(3 άξονες x 2 byte ανά άξονα = 6 byte) + XZ_gyro(2 άξονες x 2 byte ανά άξονα = 4 byte) = 10 byte ανά περίοδο.  
      
      Υπάρχει όμως η περίπτωση τη στιγμή που διαβάζουμε από την στοίβα, να γράφονται την ίδια στιγμή νέα δεδομένα σε αυτήν. Γι αυτό υπάρχει έλεγχος σε περίπτωση που τα δεδομένα που θα διαβαστούν, δεν αποτελούνται από έναν ακέραιο αριθμό δειγμάτων. Σχετικά με το προηγούμενο παράδειγμα, αν ο αισθητήρας μας πεί όταν τον ρωτήσουμε ότι έχει 46 byte έτοιμα να διαβαστούν, τότε θα διαβαστούν τα 46 - 46%10 = 46 - 6 = 40 byte. Τα άλλα έξη θα μείνουν αδιάβσστα ώστε στην επόμενη επανάληψη να συμπληρώσουν τον αριθμό τον σωστό αριθμό byte που αποτελείται το δείγμα μας, δηλαδή 10 byte.   
      
   * __private: parseSensorValues():__   Διαβάζει όλες τις τιμές της στοίβας και ανανεώνει τα αντίστοιχα πεδία με τον μέσο όρο των τιμών κάθε μέτρησης
   * __refresh(int millis):__   Καλεί την parseSensorValues()
   * __getMaxDT():__   Επιστρέφει τον μέγιστο χρόνο (σε millisecond) κατά τον οποίο πρέπει να καλείται η parseSensorValues() πρίν χαθούν παλιές μετρήσεις
   * __getFIFOSampleSize():__ Επιστρέφει τον αριθμό των byte που αποτελούν __ένα__ ακέραιο δείγμα.
   * __getFIFOEnabledSensors():__ Επιστρέφει τον αριθμό των αισθητήρων που είναι ενεργοποιημένοι.
   
4. ### Performance
   floating point calculations  
   Επειδή όμως δεν θέλουμε να χρησιμοποιήσουμε μεταβλητές κινητής υποδιαστολής για τις πράξεις στο Arduino, θα χρησιμοποιήσουμε μεταβλητές τύπου long, για μέγιστη ακρίβεια και ταχύτητα. Μια μεταβλητή long των 32bit μπορεί να αποθυκεύσει τιμές από -2,147,483,648 μέχρι 2,147,483,647. Αυτό πρακτικά σημαίνει ότι η αρκίβεια για περιστροφή από -180° μέχρι 180° θα είναι 1/10,000,000 της μοίρας.
   
   SRAM optimization 
   https://learn.adafruit.com/memories-of-an-arduino/optimizing-sram

## Configuration
