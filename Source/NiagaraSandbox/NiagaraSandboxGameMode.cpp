// Copyright Epic Games, Inc. All Rights Reserved.

#include "NiagaraSandboxGameMode.h"
#include "NiagaraSandboxCharacter.h"
#include "UObject/ConstructorHelpers.h"

ANiagaraSandboxGameMode::ANiagaraSandboxGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPersonCPP/Blueprints/ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
