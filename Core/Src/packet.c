int Ascii_to_Int(char number)
{
    for(int i = 48; i < 48+10; i++)
        if(number == i)
            return (i-48);
    return 0;
}

float convert(char* value)
{
    return (Ascii_to_Int(value[1])*10)+Ascii_to_Int(value[2])
            + (Ascii_to_Int(value[4])*0.1)
            + (Ascii_to_Int(value[5])*0.01)
            + (Ascii_to_Int(value[6])*0.001)
            + (Ascii_to_Int(value[7])*0.0001);
}
