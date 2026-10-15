#include "GncBytesLibrary.h"

#include "Misc/FileHelper.h"

FString UGncBytesLibrary::BytesToString(const TArray<uint8>& Bytes)
{
	FString Text;
	FFileHelper::BufferToString(Text, Bytes.GetData(), Bytes.Num());
	return Text;
}

TArray<uint8> UGncBytesLibrary::StringToBytes(const FString& Text)
{
	const FTCHARToUTF8 Utf8(*Text);
	TArray<uint8> Bytes;
	Bytes.Append(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
	return Bytes;
}
