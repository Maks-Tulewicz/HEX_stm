#!/bin/bash

echo "================================"
echo " Hexapod Documentation Builder"
echo "================================"
echo

# Sprawdź czy Doxygen jest zainstalowany
if ! command -v doxygen &> /dev/null; then
    echo "❌ BŁĄD: Doxygen nie jest zainstalowany!"
    echo
    echo "Zainstaluj Doxygen:"
    echo "  Ubuntu/Debian: sudo apt-get install doxygen graphviz"
    echo "  macOS:         brew install doxygen graphviz"
    echo "  CentOS/RHEL:   sudo yum install doxygen graphviz"
    echo
    exit 1
fi

echo "✅ Doxygen znaleziony: $(doxygen --version)"

# Sprawdź czy Graphviz jest dostępny (opcjonalnie)
if command -v dot &> /dev/null; then
    echo "✅ Graphviz znaleziony: $(dot -V 2>&1 | head -n1)"
else
    echo "⚠️  Graphviz nie znaleziony - diagramy nie będą generowane"
fi
echo

# Sprawdź czy istnieje Doxyfile
if [ ! -f "Doxyfile" ]; then
    echo "❌ BŁĄD: Brak pliku Doxyfile!"
    echo "Upewnij się, że uruchamiasz skrypt w katalogu projektu."
    echo
    exit 1
fi

echo "✅ Doxyfile znaleziony"
echo

# Stwórz katalog docs jeśli nie istnieje
if [ ! -d "docs" ]; then
    echo "📁 Tworzenie katalogu docs..."
    mkdir -p docs
fi

echo "✅ Katalog docs gotowy"
echo

# Uruchom Doxygen
echo "🔄 Generowanie dokumentacji..."
echo
doxygen Doxyfile

if [ $? -eq 0 ]; then
    echo
    echo "================================"
    echo "   DOKUMENTACJA WYGENEROWANA!"
    echo "================================"
    echo
    echo "📄 Dokumentacja dostępna w: docs/html/index.html"
    echo
    
    # Zapytaj czy otworzyć dokumentację
    read -p "🌐 Czy otworzyć dokumentację w przeglądarce? (t/n): " open
    case $open in
        [Tt]* | [Yy]* | tak | yes )
            if command -v xdg-open &> /dev/null; then
                xdg-open docs/html/index.html
            elif command -v open &> /dev/null; then
                open docs/html/index.html
            else
                echo "⚠️  Nie można otworzyć przeglądarki automatycznie"
                echo "   Otwórz ręcznie: docs/html/index.html"
            fi
            ;;
        * )
            echo "📝 Dokumentacja gotowa do przeglądania"
            ;;
    esac
else
    echo
    echo "================================"
    echo "     BŁĄD GENEROWANIA!"
    echo "================================"
    echo
    echo "❌ Sprawdź błędy powyżej i popraw kod."
    echo
    exit 1
fi

echo
echo "✅ Gotowe!"