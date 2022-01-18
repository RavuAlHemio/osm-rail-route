# Michelbeuern (gleich vor dem Anschlussgleis) -> Hauptwerkstätte
#& cargo run --release "C:\Users\Ondra\Downloads\vienna-trams-latest.osm.pbf" `
#    48.2219902 16.3439850 `
#    48.1502375 16.4565878

# Rodaun -> Stammersdorf
& cargo run --release -- `
    route `
    "C:\Users\Ondra\Downloads\vienna-trams-latest.osm.pbf" `
    48.1336924 16.2665872 `
    48.2983866 16.4197188
