def getLocationInfo(location):
    locations = {
        'Tutankhamun': "Tutankhamun was an ancient Egyptian pharaoh",
        'Ramses II': "Ramses II, also known as Ramses the Great, was an ancient Egyptian pharaoh who reigned from 1279 BCE to 1213 BCE. He is known for his military campaigns, building projects, and numerous statues and temples erected in his honor, including the iconic Abu Simbel temples.",
        'Seti I': "Seti I was an ancient Egyptian pharaoh who ruled from 1290 BCE to 1279 BCE. He is known for his military campaigns and building projects, including the construction of the grand temple of Abydos, which features impressive reliefs and carvings. Seti I was also the father of Ramses II, one of the most famous pharaohs in Egyptian history.",
        'Hatshepsut': "Hatshepsut was an ancient Egyptian queen who ruled as pharaoh from approximately 1479 BCE to 1458 BCE. She is known for her building projects, including the construction of the famous mortuary temple at Deir el-Bahri, and for her trade expeditions to the Land of Punt. Hatshepsut is also notable for being one of the few women to have ruled Egypt as pharaoh.",
        'Thutmose III': "Thutmose III was an ancient Egyptian pharaoh who reigned from approximately 1479 BCE to 1425 BCE. He is known for his military campaigns, which expanded the Egyptian empire to its greatest extent, and for his building projects, including the temple complex at Karnak. Thutmose III is considered one of the greatest pharaohs in Egyptian history and is often referred to as the Napoleon of Egypt.",
        'Amenhotep III': "Amenhotep III was an ancient Egyptian pharaoh who ruled from approximately 1386 BCE to 1353 BCE. He is known for his extensive building projects, including the Temple of Luxor and the Colossi of Memnon, and for his diplomatic relationships with other kingdoms. Amenhotep III was also known for his lavish lifestyle and his collection of exotic animals. His reign is often considered a time of peace and prosperity in ancient Egypt.",
        'Akhenaten': "Akhenaten was an ancient Egyptian pharaoh who ruled from approximately 1353 BCE to 1336 BCE. He is known for his religious reforms, which introduced the worship of a single god, the sun disc Aten, and for his construction of a new capital city, Amarna. Akhenaten's reign marked a significant departure from traditional Egyptian religion and culture, and his reforms were eventually reversed after his death. He is often considered one of the most controversial and enigmatic pharaohs in Egyptian history.",
        'Ramesses III': "Ramesses III was an ancient Egyptian pharaoh who reigned from approximately 1186 BCE to 1155 BCE during the New Kingdom period. He is known for his military campaigns against foreign invaders, including the Sea Peoples, and for his building projects, including the temple complex at Medinet Habu. Ramesses III also had a harem of wives and numerous children, and his reign was marked by political turmoil and conspiracies against him, including an assassination attempt. Despite these challenges, Ramesses III is often considered one of the last great pharaohs of Egypt.",
        'Merneptah': "Merneptah was an ancient Egyptian pharaoh who ruled from approximately 1213 BCE to 1203 BCE during the New Kingdom period. He is known for his military campaigns, including his victory over the invading Libyans and his mention of a campaign against the Israelites in his famous victory stele. Merneptah also undertook several building projects, including the construction of a mortuary temple in his honor."
    }
    # 'Tutankhamun': "Tutankhamun was an ancient Egyptian pharaoh who ruled from approximately 1332 BCE to 1323 BCE. He is famous for his tomb, which was discovered almost entirely intact in 1922 by archaeologist Howard Carter, and the wealth of treasures and artifacts found within it.",

    return locations[location]


# NOTE: locations name MUST be exactly the same as the
# keys of the locations above
LocationsNames = ['Tutankhamun', 'Ramses II', 'Seti I',
                  'Hatshepsut', 'Thutmose III', 'Amenhotep III',
                  'Akhenaten', 'Ramesses III', 'Merneptah']


# the threshold for when to start recording
# the question as the speaker starts to speak
SpeakingThreshold = 15

TTS_API_KEY = 'b2c64b49002c6fe8c52d09d16d046ec5'

# Change the name to the site name
SiteName = "Egyptian Museum"
StartPrompt = "Act as a tour guide with the name Teaty in the {}. Answer the following question in no more than 3 sentence".format(
    SiteName)
