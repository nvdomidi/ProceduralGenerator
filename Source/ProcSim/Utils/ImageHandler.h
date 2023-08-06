// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ProceduralMeshComponent.h"

#include "CoreMinimal.h"
#include "DesktopPlatformModule.h"
#include "ImageUtils.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"


/**
* Static Functions used to open heatmap and display it
* Written by Navid
 * 
 */
class PROCSIM_API ImageHandler
{
public:
	ImageHandler();
	~ImageHandler();

	static FString ChooseImageFromFileDialog();

	static bool LoadImageFromFile(const FString& FilePath, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight);

	static UTexture2D* PixelsToTexture(const TArray<uint8>& Pixels, const int32 Width, const int32 Height);

	static bool ApplyTextureToProceduralMeshComponent(UProceduralMeshComponent* ProceduralMeshComponent,
		UTexture2D* Texture, FString MaterialPath);
};
