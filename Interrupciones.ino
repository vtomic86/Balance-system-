
/*
 *  interrupciones para la cuadratura de encoders
 *  
 */
void motorIzqInterA()
{
  
  _IzqEncoderBSet = digitalReadFast(e_EncoderIzqPinB);   // lee en pin de entrada

  //y ajusta el conteo 
#ifdef encoderIzquierdoReversa
  _ticksIzqEncoder -= _IzqEncoderBSet ? -1 : +1;
#else
  _ticksIzqEncoder += _IzqEncoderBSet ? -1 : +1;
#endif
}

void motorDerInterA()
{
  
  _DerEncoderBSet = digitalReadFast(e_EncoderDerPinB);  


#ifdef RightEncoderIsReversed
  _ticksDerEncoder -= _DerEncoderBSet ? -1 : +1;
#else
  _ticksDerEncoder += _DerEncoderBSet ? -1 : +1;
#endif
}

