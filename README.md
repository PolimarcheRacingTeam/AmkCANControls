# AmkCANControls
Progetto Arduino CAN Bus


Questo progetto Arduino utilizza la libreria CANSAME5x per impostare una comunicazione CAN (Controller Area Network).
Il codice definisce diverse variabili per l'invio e la ricezione di messaggi CAN, nonché funzioni per la costruzione e l'invio di messaggi CAN e per la gestione dei messaggi ricevuti.

Funzioni
setup()
La funzione setup() inizializza la comunicazione seriale, 
imposta il bus CAN a 500 kbps e imposta una funzione di callback receive_message() da chiamare ogni volta che viene ricevuto un messaggio CAN.

loop()
La funzione loop() costruisce un messaggio CAN utilizzando la funzione build_message(),
e lo invia a due nodi diversi utilizzando la funzione send_message(). Quindi controlla se i messaggi sono stati inviati correttamente.

receive_message()
La funzione receive_message() viene chiamata ogni volta che viene ricevuto un messaggio CAN. 
Legge l'ID del messaggio e i dati, e stampa i dati in una stringa formattata. Controlla anche l'ID del messaggio per determinare da quale nodo proviene e che tipo di dati contiene.

build_message()
La funzione build_message() accetta diversi parametri (control, target_velocity, torque_limit_positive e torque_limit_negative) 
e costruisce un messaggio CAN con quei parametri nel formato corretto.

send_message()
La funzione send_message() accetta un messaggio e un indirizzo di nodo, e invia il messaggio a quel nodo utilizzando il bus CAN.

Utilizzo
Per utilizzare questo progetto, è necessario caricare il codice su una scheda Arduino compatibile e collegarla a un bus CAN. 
È possibile utilizzare la funzione Serial.print() per visualizzare i messaggi inviati e ricevuti sulla porta seriale.
