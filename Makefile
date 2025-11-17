# The cleanym2bits generic makefile

CC     = gcc
LIBS   = -lsndfile -lsamplerate -lm
DEBUG  = -Wall -g
 
all: cleanym2bits_encoder cleanym2bits_decoder
 
cleanym2bits_encoder:	cleanym2bits_encoder.c
	$(CC) cleanym2bits_encoder.c $(DEBUG) $(LIBS) -o cleanym2bits_encoder

cleanym2bits_decoder:	cleanym2bits_decoder.c
	$(CC) cleanym2bits_decoder.c $(DEBUG) $(LIBS) -o cleanym2bits_decoder

clean:
	rm -f *.o cleanym2bits_encoder cleanym2bits_decoder

