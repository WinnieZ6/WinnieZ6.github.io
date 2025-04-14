import React, { useState } from 'react';

const GeoGame = () => {
  // Your question pool (add more questions as needed)
  const allQuestions = [
    "Discuss the relationships between globalisation and human development [16]",
    "(a) Using examples, analyse the influence of diaspora populations on the cultural identity of different places. [12]",
    "Analyse the reasons why people’s freedom to participate in global interactions varies from place to place. [12]",
    "Using examples, analyse the reasons why some national governments introduce trade restrictions.",
    "Explain how microfinance organizations and alternative trading networks can help communities to develop. [12]",
    "Using examples, analyse ways in which global interactions can lead to improved gender equality. [12]",
    " Discuss the relationships between globalisation and human development [16]",
    "Analyse the validity and reliability of two indicators of human development [12]"
    // "",
    // "",
    // "  ",
    // "",
    // "",
    // "",
    // ""
    // Add additional questions here
  ];

  // Set up state for the remaining questions and current question
  const [remaining, setRemaining] = useState([...allQuestions]);
  const [currentQuestion, setCurrentQuestion] = useState(null);

  // Function to draw a random question
  const drawQuestion = () => {
    if (remaining.length === 0) {
      setCurrentQuestion("All questions have been drawn. Please restart the game.");
      return;
    }
    const randomIndex = Math.floor(Math.random() * remaining.length);
    const selected = remaining[randomIndex];
    setCurrentQuestion(selected);
    const newRemaining = [...remaining];
    newRemaining.splice(randomIndex, 1);
    setRemaining(newRemaining);
  };

  // Function to reset the game
  const resetGame = () => {
    setRemaining([...allQuestions]);
    setCurrentQuestion(null);
  };

  return (
    <div className="flex flex-col items-center justify-center h-screen bg-black relative">
      {/* Slot Machine Animation */}
      <div className="relative w-80 h-80 flex justify-center items-center bg-gray-800 rounded-lg shadow-xl mb-8 overflow-hidden">
        {/* Spinning gradient background */}
        <div className="absolute inset-0 bg-gradient-to-r from-yellow-300 via-red-300 to-purple-400 animate-spin"></div>
        {/* Concentric circles for effect */}
        <div className="absolute w-32 h-32 bg-white rounded-full"></div>
        <div className="absolute w-24 h-24 bg-gray-200 rounded-full"></div>
        <div className="absolute w-16 h-16 bg-gray-800 rounded-full"></div>
      </div>

      {/* Display current question */}
      <div className="text-3xl font-bold text-white bg-black bg-opacity-50 px-8 py-4 rounded-lg mb-8 text-center">
        {currentQuestion || "Press 'Draw Question' to begin"}
      </div>

      {/* Action buttons */}
      <div className="flex space-x-4">
        <button 
          onClick={drawQuestion} 
          className="px-6 py-3 bg-yellow-500 text-white font-bold rounded-lg shadow-lg hover:bg-yellow-600 transition"
        >
          Draw Question
        </button>
        <button 
          onClick={resetGame} 
          className="px-6 py-3 bg-blue-500 text-white font-bold rounded-lg shadow-lg hover:bg-blue-600 transition"
        >
          Reset
        </button>
      </div>
    </div>
  );
};

export default GeoGame;
