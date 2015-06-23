// I think that this should change the LED light to green.
// need to change things from m_platform and do open port blah blah initialization

    // Make the Head LED to green
	//WriteWord(CM730::ID_CM, CM730::P_LED_HEAD_L, MakeColor(0, 255, 0), 0);
	unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
	m_Platform->WritePort(txpacket, 9);