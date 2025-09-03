# Project Goal

The goal of this project is to create a Procedural City Generator in Unreal Engine 5. This project will be created as a plugin and can serve as a tool for rapidly creating procedural scenes for games, simulations, etc. The city generator will be customizable to a degree so that the user can generate the style of city they want. The project will also include a real city importer, where users can import real city data from OpenStreetMap. The generator will create a city similar to the one imported in the same fashion as the random generated ones.

# High Level Approach

1. Creating a Procedural Road Generator. This generated road network will serve as the framework for placing different city elements.
2. Extracting planar cycles from the graph. These areas will be used to place different element of the city.
3. Parceling the different areas. Each cycle must be subdivided into smaller parcels. Each parcel will have one city element placed inside of it.
4. Creating random city elements in blueprints. These will be customizable random little areas like buildings, schools, parking lots, plazas, parks, hospitals, etc.
5. Filling the parcels with the city elements. Sizing the elements to fit the parcels and not intersect with the road is important in this step.
6. Random height for the ground. The ground must be customizable to different heights. Objects placed inside of it must be adjusted to the proper height and angle.
7. City details: intersections, sidewalks, bridges, traffics lights, etc. These must be shown in the correct place.
8. Integrating real world data. Importing data from OpenStreetMap must be possible. The algorithm must generate a city similar to the imported data.
9. Creating city elements related to OpenStreetMap that dont exist in our procedural generator such as curved roads, bridges, etc.
