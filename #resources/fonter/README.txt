Thanks for this tool Charles! (or maybe it's not originaly created by Charles?? but my search all over the web of who could be the original creator of fonter.c produced no results)
https://github.com/cnlohr/pylotron/blob/master/fonter.c (turn PBM images of fonts into RREs C arrays - used to create wlcd_font_*.h files)
--------------------

compile fonter tool:
gcc fonter.c -o fonter

--------------------

create font_term_8x16_cp437.h:
fonter font_term_8x16_cp437.pbm 8 16 > font_term_8x16_cp437.h

--------------------

*.h description:

uint16_t font_places[] - offsets to array font_pieces[]
example for char with ASCII 65 ("A"):
font_places[65] = 0x0170
font_places[66] = 0x0177
=> for drawing char "A" we'll use 7 rectangles, first rectangle is here -> font_pieces[0x0170]

uint16_t font_pieces[] - each 16-bit value describes one rectangle
0 b 0000 0000 0000 0000
      H    W    Y    X
note: add 1 to W, H to get real width, height
