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

# Project Timing

The deadline of this project is on April 19th 2024.

Estimated timing for each step:

* Step 1 has been completed before.
* Step 2 = 1 week to complete.
* Step 3 = 2 weeks to complete.
* Step 4 = 6 weeks to complete.
* Step 5 = 3 weeks to complete.
* Step 6 = 3 weeks to complete.
* Step 7 = 4 weeks to complete.
* Step 8 = 6 weeks to complete.
* Step 9 = 3 weeks to complete.
* Cleanup and finalizing = 4 weeks to complete.

Estimated timing = 224 days. This project needs more than 1 person to work on it to reach the deadline.

# Launch Plan

| Target Date    | Milestone    | Description    | Exit Criteria    |
|----------------|--------------|----------------|------------------|
| 2023-12-09 | step 2 | road generation + planar cycles + tests | tests complete + meeting with team |
| 2023-12-23 | step 3 | parceling done | no bugs on step 1, 2, 3 + meeting with team |
| 2024-01-06 | step 4 | step 4 and merge code with team | you have their codebase & blueprints + meeting |
| 2024-01-20 | step 5 | step 5 and code base is merged | procedural city is being generated + meeting |
| 2024-02-10 | step 6 | step 6 + tests | city with random height + meeting |
| 2024-03-09 | step 7 | step 7 + city looks good | no bugs + meeting |
| 2024-03-30 | step 8 | step 8 and merge code with team | city gets generated from openstreetmap + meeting |
| 2024-04-19 | step 9 | step 9 and merge code complete | openstreetmap + all details + meeting |
| 2024-04-27 | step 8 | step 8 and merge code with team | city gets generated from openstreetmap |

# Project Management

This project requires me to be very disciplined to complete as I am extremely busy at the moment. I will try to carefully plan my days to reach deadlines. I will try to hire someone to help me if I start running late for my deadlines.

## Planning Tasks

* Tasks will be created in detail, in (maybe) 2 week sprints and added to my Trello board. 
* Milestones will be created for each step of the project. Each milestone will have Trello tasks and github issues.
* Trello tasks is used for deadlines. Github issues is used for tracking the details of everything done. 

## Daily Schedule

I have set timers to manage my hours every day. Each day starts with me planning what I will do for the day. Each day will end with me assessing my progress that day.

* 6:20 AM = Wake up & do prayers
* 7:20 AM = Plan my day
* 7:30 AM = Start working on this project
* 8:30 AM = Rest
* 8:45 AM = Start working on your job
* 6:15 PM = End working on your job
* 11:30 PM = Assess your day

## Motivation

> "The magic you are looking for is in the work you are avoiding." - Unknown
> "I find that the harder I work, the more luck I seem to have." – Thomas Jefferson