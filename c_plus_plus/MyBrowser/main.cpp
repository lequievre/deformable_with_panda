#include "ExampleBrowser/OpenGLExampleBrowser.h"

#include "Bullet3Common/b3CommandLineArgs.h"
#include "Utils/b3Clock.h"

#include "ExampleEntries.h"
#include "Bullet3Common/b3Logging.h"

#include "LinearMath/btAlignedAllocator.h"

static double gMinUpdateTimeMicroSecs = 1000.;

static OpenGLExampleBrowser* sExampleBrowser = 0;


int main(int argc, char* argv[])
{
	{
		b3CommandLineArgs args(argc, argv);
		b3Clock clock;
		args.GetCmdLineArgument("minUpdateTimeMicroSecs", gMinUpdateTimeMicroSecs);

		ExampleEntriesAll examples;
		examples.initExampleEntries();
	
		sExampleBrowser = new OpenGLExampleBrowser(&examples);
		
		bool init = sExampleBrowser->init(argc, argv);
		
		clock.reset();
		if (init)
		{
			do
			{
				float deltaTimeInSeconds = clock.getTimeMicroseconds() / 1000000.f;
				if (deltaTimeInSeconds > 0.1)
				{
					deltaTimeInSeconds = 0.1;
				}
				if (deltaTimeInSeconds < (gMinUpdateTimeMicroSecs / 1e6))
				{
					b3Clock::usleep(gMinUpdateTimeMicroSecs / 10.);
				}
				else
				{
					clock.reset();
					sExampleBrowser->update(deltaTimeInSeconds);
				}
			} while (!sExampleBrowser->requestedExit());
		}
		
		delete sExampleBrowser;
		
	}
    
	return 0;
}
