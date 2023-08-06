// Fill out your copyright notice in the Description page of Project Settings.


#include "ImageHandler.h"
#include "Engine/TextureRenderTarget2D.h"

ImageHandler::ImageHandler()
{
}

ImageHandler::~ImageHandler()
{
}

FString ImageHandler::ChooseImageFromFileDialog()
{
	// Create a file dialog to select the image file
	IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
	if (DesktopPlatform != nullptr)
	{
        TArray<FString> OutFileNames;
        bool bOpened = DesktopPlatform->OpenFileDialog(
            nullptr,
            TEXT("Select Image"),
            FPaths::GetProjectFilePath(),
            TEXT(""),
            TEXT("Image Files|*.png;*.jpg;*.bmp"),
            EFileDialogFlags::None,
            OutFileNames
        );

        if (OutFileNames.IsValidIndex(0))
            return OutFileNames[0];
	}

	return FString("");
}

bool ImageHandler::LoadImageFromFile(const FString& FilePath, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight)
{
    // Load the image from the file
    TArray<uint8> FileData;
    if (!FFileHelper::LoadFileToArray(FileData, *FilePath))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load image from file: %s"), *FilePath);
        return false;
    }

    // Create an image wrapper (you can choose different formats based on the image type)
    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
    EImageFormat ImageFormat = ImageWrapperModule.DetectImageFormat(FileData.GetData(), FileData.Num());

    TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(ImageFormat);
    if (!ImageWrapper.IsValid() || !ImageWrapper->SetCompressed(FileData.GetData(), FileData.Num()))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create image wrapper or set compressed data for image: %s"), *FilePath);
        return false;
    }

    // Get the width, height, and pixel data
    OutWidth = ImageWrapper->GetWidth();
    OutHeight = ImageWrapper->GetHeight();
    TArray<uint8> RawPixels;
    
    if (!ImageWrapper->GetRaw(ERGBFormat::Gray, 8, RawPixels))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get raw image data: %s"), *FilePath);
        return false;
    }

    // Copy pixel data row-wise into the output array (OutPixels)
    OutPixels.Reset();
    OutPixels.Append(RawPixels);

    UE_LOG(LogTemp, Warning, TEXT("The number of rawpixels are: %d"), RawPixels.Num());

    return true;
}

UTexture2D* ImageHandler::PixelsToTexture(const TArray<uint8>& Pixels, const int32 Width, const int32 Height)
{
    if (Pixels.Num() == 0) {
        UE_LOG(LogTemp, Warning, TEXT("Nothing in the pixels!"));
        return nullptr;
    }

    // We need to repeat the pixels four times to make it BGRA format
    TArray<uint8> coloredPixels{};
    for (auto pixel : Pixels) {
        for (int i = 0; i < 4; i++)
            coloredPixels.Add(pixel);
    }

    UTexture2D* Texture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);

    uint8* TextureData = (uint8*)Texture->PlatformData->Mips[0].BulkData.Lock(LOCK_READ_WRITE);

    // Copy the pixel data to the texture
    FMemory::Memcpy(TextureData, coloredPixels.GetData(), coloredPixels.Num());

    // Unlock the texture
    Texture->PlatformData->Mips[0].BulkData.Unlock();

    // Update the texture's properties
    Texture->UpdateResource();

    return Texture;

}

bool ImageHandler::ApplyTextureToProceduralMeshComponent(UProceduralMeshComponent* ProceduralMeshComponent,
    UTexture2D* Texture, FString MaterialPath)
{
    UObject* LoadedMaterial = StaticLoadObject(UMaterialInterface::StaticClass(), NULL, *MaterialPath);

    // Check if the material is successfully loaded
    if (LoadedMaterial && LoadedMaterial->IsA(UMaterialInterface::StaticClass()))
    {
        UMaterialInterface* MaterialInterface = Cast<UMaterialInterface>(LoadedMaterial);
        // Create a dynamic material instance from the loaded material
        UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(MaterialInterface, nullptr);

        if (DynamicMaterial)
        {
            DynamicMaterial->SetTextureParameterValue(FName("Texture"), Texture);
            // Set the dynamic material instance to the mesh component
            ProceduralMeshComponent->SetMaterial(0, DynamicMaterial);

            return true;
        }
    }

    return false;
}
