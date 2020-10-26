% % Program to do text to speech.
% % Get user's sentence
% userPrompt = 'What do you want the computer to say?';
% titleBar = 'Text to Speech';
% defaultString = 'Hello World!  MATLAB is an awesome program!';
% caUserInput = inputdlg(userPrompt, titleBar, 1, {defaultString});
% if isempty(caUserInput)
% 	return;
% end; % Bail out if they clicked Cancel.
% caUserInput = char(caUserInput); % Convert from cell to string.
% NET.addAssembly('System.Speech');
% obj = System.Speech.Synthesis.SpeechSynthesizer;
% obj.Volume = 100;
% Speak(obj, caUserInput);


%% USES:
% https://www.mathworks.com/matlabcentral/fileexchange/18091-text-to-speech

% Speak the text; 
% tts('I can speak.'); 
% List availble voices; 
% tts('I can speak.','List'); 
tts('I can speak. Here is a longer test','Microsoft Zira Desktop - English (United States)'); 
% Do not speak out, store the speech in a variable; 
% w = tts('I can speak.',[],-4,44100); 
% wavplay(w,44100);