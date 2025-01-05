/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util;

/**
 * Represents a game element (measurements are in meters)
 *
 * @param name The name of the object
 * @param length The length of the object
 * @param width The width of the object
 * @param height The height of the object (floor to middle)
 */
public record GameElement(String name, double length, double width, double height) {}
